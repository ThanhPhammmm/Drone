#include "altitude_estimator_task.h"

#define ALT_EST_BARO_MAX_AGE_US     30000
#define ALT_EST_RANGE_MAX_AGE_US    60000

#define ALT_EST_PERIOD_S            0.01f     /* run the filter on every 10 ms of samples */
#define ALT_EST_GAP_MAX_S           0.05f     /* longer gap between samples: start the window over */
#define ALT_EST_LOSS_CYCLES         50U       /* 0.5 s with no height sensor: not valid */

#define ALT_EST_RANGE_MIN_M         0.02f
#define ALT_EST_RANGE_MAX_M         3.5f
#define ALT_EST_RANGE_TILT_COS      0.866f    /* more than 30 deg of tilt: the beam misses the floor below */
#define ALT_EST_RANGE_STD_M         0.03f     /* 1 sigma the filter gives a range height... */
#define ALT_EST_RANGE_STD_PER_M     0.02f     /* ...plus this per metre: it gets noisier far away */
#define ALT_EST_RANGE_REJECTS_MAX   10U       /* outliers in a row: the floor below changed height */
#define ALT_EST_FLOOR_MARGIN_M      0.02f     /* range within this of the standing reading = on a floor */
#define ALT_EST_FLOOR_FRESH_CYCLES  30U       /* range sample at most this old (x 10 ms) for onFloor: rides out a few invalid readings near the sensor minimum */
#define ALT_EST_FLOOR_LIFT_VZ       0.05f     /* lift-off speed cap while the range says "on the floor" */

#define LIFT_BASE_TAU_S             0.5f      /* s, standing reading, learned while the motors do not fly */
#define LIFT_LEAK_TAU_S             1.0f      /* s, forgets a small accelerometer error instead of adding it up */

AltitudeEstimator_Handle_t altitudeEstimator;
static AltKF_t altKF;

void AltitudeEstimator_SetTaskHandle(TaskHandle_t handle){
	altitudeEstimator.altitudeTask = handle;
}

/* 1 = a new range sample that can be used; h = height above the floor below */
static uint8_t AltEst_RangeHeight(const Attitude_Data_t *attitude, uint32_t *lastUs, float *h){
    Range_Data_t range;
    if(RangeTopic_Copy(&range, 0) != pdPASS || range.timestamp_us == 0) return 0;
    if(range.timestamp_us == *lastUs) return 0;
    *lastUs = range.timestamp_us;

    int32_t age = Time_DiffUs(attitude->timestamp_us, range.timestamp_us);
    if(age >= ALT_EST_RANGE_MAX_AGE_US || age <= -ALT_EST_RANGE_MAX_AGE_US) return 0;
    if(!range.valid) return 0;
    if(range.distance_m < ALT_EST_RANGE_MIN_M || range.distance_m > ALT_EST_RANGE_MAX_M) return 0;

    /* body z seen from the earth frame: cos(roll) * cos(pitch) */
    float tiltCos = attitude->q0*attitude->q0 - attitude->q1*attitude->q1
                  - attitude->q2*attitude->q2 + attitude->q3*attitude->q3;
    if(tiltCos < ALT_EST_RANGE_TILT_COS) return 0;

    *h = range.distance_m * tiltCos;
    return 1;
}

void AltitudeEstimatorTask(void *argument){
	AltitudeEstimator_SetTaskHandle(xTaskGetCurrentTaskHandle());
	AttitudeTopic_Subscribe(altitudeEstimator.altitudeTask, ALTITUDE_ESTIMATOR_ID_TASK);

    Attitude_Data_t attitude;
    Baro_Data_t     baro;

    uint8_t  initialized    = 0;
    uint16_t lostCycles     = ALT_EST_LOSS_CYCLES;
    uint32_t lastBaroUs     = 0;

    uint32_t lastRangeUs    = 0;
    uint8_t  rangeAnchored  = 0;
    float    rangeOffset    = 0.0f;   /* m, z of the floor below: z = h + rangeOffset */
    float    rangeStanding  = 0.0f;   /* m, h while standing: the sensor's own height */
    uint8_t  rangeStandSet  = 0;
    uint8_t  onFloor        = 0;
    uint16_t rangeRejects   = 0;
    uint16_t rangeUsedAgo   = ALT_EST_LOSS_CYCLES;   /* cycles since the filter took a range sample */
    uint16_t rangeSeenAgo   = ALT_EST_LOSS_CYCLES;   /* cycles since a usable range sample came */
    float    agl            = 0.0f;

    uint32_t lastSampleUs = 0;
    uint8_t  haveSample   = 0;
    float    dtAcc        = 0.0f;
    float    accelUpAcc   = 0.0f;

    float    liftBase     = 0.0f;   /* m/s^2, accelUp while standing */
    uint8_t  liftBaseSet  = 0;
    float    liftVzRaw    = 0.0f;   /* m/s, accelerometer-only lift-off speed */
    float    liftVz       = 0.0f;   /* the same, held down while the range sensor still reads the floor */

    while(1){
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if(AttitudeTopic_Copy(&attitude) != pdPASS) continue;

        if(!haveSample){
            lastSampleUs = attitude.timestamp_us;
            haveSample   = 1;
            continue;
        }

        if(attitude.timestamp_us == lastSampleUs) continue;
        float sampleDt = (float)(uint32_t)(attitude.timestamp_us - lastSampleUs) * 1e-6f;
        lastSampleUs = attitude.timestamp_us;

        if(sampleDt > ALT_EST_GAP_MAX_S){
            dtAcc      = 0.0f;
            accelUpAcc = 0.0f;
            continue;
        }

        dtAcc      += sampleDt;
        accelUpAcc += attitude.accelUp * sampleDt;
        if(dtAcc < ALT_EST_PERIOD_S) continue;

        float dt      = dtAcc;
        float accelUp = accelUpAcc / dtAcc;
        dtAcc      = 0.0f;
        accelUpAcc = 0.0f;

        if(!Arm_MotorsAllowed()){
            if(!liftBaseSet){
                liftBase    = accelUp;
                liftBaseSet = 1;
            }
            liftBase += (accelUp - liftBase) * dt / (LIFT_BASE_TAU_S + dt);
            liftVzRaw = 0.0f;
        }
        else if(!Arm_IsAirborne()){
            liftVzRaw += (accelUp - liftBase) * dt;

            /* If the aircraft only vibrates on the ground without taking off,
            integration errors can accumulate over time and falsely indicate
            that it is flying. A leak continuously reduces the accumulated
            value toward zero, cancelling out mechanical vibration effects.*/
            liftVzRaw -= liftVzRaw * dt / LIFT_LEAK_TAU_S;
        }
        else{
            liftVzRaw = 0.0f;
        }

        /* the range sensor still reads its standing height: not off the
         * ground, whatever the vibration makes the accelerometer say */
        liftVz = (onFloor && liftVzRaw > ALT_EST_FLOOR_LIFT_VZ) ? ALT_EST_FLOOR_LIFT_VZ : liftVzRaw;

        /* baro: fresh = recent enough to call the height valid,
         *       new   = a sample the filter has not used yet */
        uint8_t baroFresh = 0;
        uint8_t baroNew   = 0;
        if(BaroTopic_Copy(&baro, 0) == pdPASS && baro.timestamp_us != 0){
            int32_t age = Time_DiffUs(attitude.timestamp_us, baro.timestamp_us);
            if(age < ALT_EST_BARO_MAX_AGE_US && age > -ALT_EST_BARO_MAX_AGE_US){
                baroFresh  = 1;
                baroNew    = (baro.timestamp_us != lastBaroUs);
                lastBaroUs = baro.timestamp_us;
            }
        }

        float h = 0.0f;
        uint8_t rangeNew = AltEst_RangeHeight(&attitude, &lastRangeUs, &h);

        if(rangeUsedAgo < ALT_EST_LOSS_CYCLES) rangeUsedAgo++;
        if(rangeSeenAgo < ALT_EST_LOSS_CYCLES) rangeSeenAgo++;
        if(rangeNew){
            rangeSeenAgo = 0;
            agl = h;
        }

        if(!initialized && baroFresh){
            AltKF_Init(&altKF, baro.altitude_m);
            initialized = 1;
        }

        if(initialized){
            uint8_t motors   = Arm_MotorsAllowed();
            uint8_t airborne = Arm_IsAirborne();

            /* the baro error (prop wash, drift) is only seen against a second
             * height: learn it only while the motors run and the range sensor is in use */
            AltKF_Predict(&altKF, accelUp, dt, motors && rangeUsedAgo < ALT_EST_LOSS_CYCLES);

            if(baroNew) AltKF_UpdateBaro(&altKF, baro.altitude_m);

            if(rangeNew){
                float z = AltKF_Get(&altKF, ALT_KF_Z);
                if(!motors || !rangeAnchored){
                    /* standing: z of the floor below is the z right now */
                    rangeOffset   = z - h;
                    rangeAnchored = 1;
                    rangeRejects  = 0;
                    if(!motors){
                        rangeStanding = h;
                        rangeStandSet = 1;
                    }
                }
                else if(AltKF_UpdateRange(&altKF, h + rangeOffset, ALT_EST_RANGE_STD_M + ALT_EST_RANGE_STD_PER_M * h)){
                    rangeRejects = 0;
                    rangeUsedAgo = 0;
                }
                else if(++rangeRejects >= ALT_EST_RANGE_REJECTS_MAX){
                    /* not a glitch, it keeps disagreeing: the floor below moved
                     * (a table, a step). Follow the new floor without a jump. */
                    rangeOffset  = z - h;
                    rangeRejects = 0;
                }
            }

            if(!airborne){
                AltKF_SetGround(&altKF, liftVz, liftBase);
            }

            if(baroNew || rangeUsedAgo == 0) lostCycles = 0;
            else if(lostCycles < ALT_EST_LOSS_CYCLES) lostCycles++;
        }

        onFloor = rangeStandSet && rangeSeenAgo < ALT_EST_FLOOR_FRESH_CYCLES &&
                  agl < rangeStanding + ALT_EST_FLOOR_MARGIN_M;

        Altitude_Data_t *alt = &altitudeEstimator.data;
        alt->altitude      = initialized ? AltKF_Get(&altKF, ALT_KF_Z)          : 0.0f;
        alt->verticalSpeed = initialized ? AltKF_Get(&altKF, ALT_KF_VZ)         : 0.0f;
        alt->accelBias     = initialized ? AltKF_Get(&altKF, ALT_KF_ACCEL_BIAS) : 0.0f;
        alt->baroBias      = initialized ? AltKF_Get(&altKF, ALT_KF_BARO_BIAS)  : 0.0f;
        alt->accelUp       = accelUp;
        alt->liftVz        = liftVz;
        alt->agl           = agl;
        alt->aglValid      = (rangeSeenAgo < ALT_EST_LOSS_CYCLES);
        alt->onFloor       = onFloor;
        alt->valid         = initialized && (lostCycles < ALT_EST_LOSS_CYCLES);
        alt->baroValid     = baroFresh;
        alt->rangeValid    = (rangeUsedAgo < ALT_EST_LOSS_CYCLES);
        alt->dt            = dt;
        alt->timestamp_us  = attitude.timestamp_us;

        AltitudeTopic_Publish(alt);
    }
}
