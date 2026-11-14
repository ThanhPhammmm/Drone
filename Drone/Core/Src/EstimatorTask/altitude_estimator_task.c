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

#define ALT_EST_COAST_AFTER_CYCLES  5U        /* no usable range sample for 50 ms (one comes every 33): coast */
#define ALT_EST_STALLED_CYCLES      400U      /* the first 4 s of it are reported as rangeStalled: long enough for
                                               * a restart, and the coast is still good to hold a spot with */
#define ALT_EST_COAST_BARO_W        0.7f      /* rad/s, how fast the baro pulls the coasted height back */
#define ALT_EST_SLOW_BIAS_TAU_S     2.0f      /* s, the filter's accel bias averaged: one reading of it is too noisy to coast on */
#define ALT_EST_SLOW_VZ_TAU_S       0.3f      /* s, the same for the climb speed the coast starts with */
#define ALT_EST_SLOW_BARO_TAU_S     0.5f      /* s, the baro height the coast measures its changes from */
#define ALT_EST_COAST_Z_STD         0.5f      /* m, m/s, m/s^2: how sure the filter is of what the coast hands back */
#define ALT_EST_COAST_VZ_STD        0.3f
#define ALT_EST_COAST_BIAS_STD      0.1f

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
    float    rangeOffset    = 0.0f; /* m, z of the floor below: z = h + rangeOffset */
    float    rangeStanding  = 0.0f; /* m, h while standing: the sensor's own height */
    uint8_t  rangeStandSet  = 0;
    uint8_t  onFloor        = 0;
    uint16_t rangeRejects   = 0;
    uint16_t rangeUsedAgo   = ALT_EST_LOSS_CYCLES;  /* cycles since the filter took a range sample */
    uint16_t rangeSeenAgo   = ALT_EST_LOSS_CYCLES;  /* cycles since a usable range sample came */
    uint16_t rangeGone      = ALT_EST_STALLED_CYCLES;   /* the same, counted on to the end of rangeStalled */

    uint8_t  slowSet        = 0;    /* the averages below are running (in flight, on the range sensor) */
    float    biasSlow       = 0.0f; /* m/s^2, the filter's accel bias, averaged */
    float    vzSlow         = 0.0f; /* m/s, its climb speed, averaged */
    float    baroSlow       = 0.0f; /* m, the baro height, averaged */
    float    zRanged        = 0.0f; /* m, the filter's height the last time a range sample set it */

    uint8_t  coasting       = 0;    /* range lost in flight: zCoast / vzCoast are the estimate */
    float    zCoast         = 0.0f;
    float    vzCoast        = 0.0f;
    float    zCoastStart    = 0.0f; /* m, where the coast began, and... */
    float    baroCoastStart = 0.0f; /* ...the baro height then */
    float    baroCoastDt    = 0.0f; /* s since the baro last corrected the coast */
    float    agl            = 0.0f; /* above ground level */

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
        if(rangeGone < ALT_EST_STALLED_CYCLES) rangeGone++;
        if(rangeNew){
            rangeSeenAgo = 0;
            rangeGone    = 0;
            agl = h;
        }

        if(!initialized && baroFresh){
            AltKF_Init(&altKF, baro.altitude_m);
            initialized = 1;
        }

        if(initialized){
            uint8_t motors   = Arm_MotorsAllowed();
            uint8_t airborne = Arm_IsAirborne();

            if(baroNew){
                if(!slowSet) baroSlow = baro.altitude_m;
                baroSlow += (baro.altitude_m - baroSlow) * 0.02f / (ALT_EST_SLOW_BARO_TAU_S + 0.02f);   /* a sample every ~20 ms */
            }

            /* in flight, the range sensor it flew on has gone quiet */
            uint8_t coastNow = airborne && rangeAnchored && slowSet && rangeGone >= ALT_EST_COAST_AFTER_CYCLES;
            if(coastNow && !coasting){
                zCoast         = zRanged;
                vzCoast        = vzSlow;
                zCoastStart    = zCoast;
                baroCoastStart = baroSlow;
                baroCoastDt    = 0.0f;
            }
            if(coasting && !coastNow){
                /* the range is back, or it has landed: the filter goes on from where the coast is */
                Kalman_SetState(&altKF.kf, ALT_KF_Z,          zCoast,   ALT_EST_COAST_Z_STD * ALT_EST_COAST_Z_STD);
                Kalman_SetState(&altKF.kf, ALT_KF_VZ,         vzCoast,  ALT_EST_COAST_VZ_STD * ALT_EST_COAST_VZ_STD);
                Kalman_SetState(&altKF.kf, ALT_KF_ACCEL_BIAS, biasSlow, ALT_EST_COAST_BIAS_STD * ALT_EST_COAST_BIAS_STD);
            }
            uint8_t coastEnded = coasting && !coastNow;
            coasting = coastNow;

            if(coasting){
                vzCoast += (accelUp - biasSlow) * dt;
                zCoast  += vzCoast * dt;

                baroCoastDt += dt;
                if(baroNew){
                    /* how far the baro has moved since the stall, against how far the coast has */
                    float e = (baro.altitude_m - baroCoastStart) - (zCoast - zCoastStart);
                    zCoast  += 2.0f * ALT_EST_COAST_BARO_W * e * baroCoastDt;
                    vzCoast += ALT_EST_COAST_BARO_W * ALT_EST_COAST_BARO_W * e * baroCoastDt;
                    baroCoastDt = 0.0f;
                }
            }
            else{
                AltKF_Predict(&altKF, accelUp, dt, motors && rangeUsedAgo < ALT_EST_LOSS_CYCLES);

                /* not in the step the range comes back in: that sample sets the height */
                if(baroNew && !coastEnded) AltKF_UpdateBaro(&altKF, baro.altitude_m);
            }

            if(rangeNew){
                float z = AltKF_Get(&altKF, ALT_KF_Z);
                if(!motors || !rangeAnchored){
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
                    /* the floor below moved (a table, a step). Follow the new floor without a jump. */
                    rangeOffset  = z - h; // set new offset
                    rangeRejects = 0;
                }
            }

            if(!airborne){
                AltKF_SetGround(&altKF, liftVz, liftBase);
                slowSet = 0;
            }
            else if(rangeUsedAgo == 0){
                /* flying on the range sensor: what a coast would start from */
                float kfBias = AltKF_Get(&altKF, ALT_KF_ACCEL_BIAS);
                float kfVz   = AltKF_Get(&altKF, ALT_KF_VZ);
                if(!slowSet){
                    biasSlow = kfBias;
                    vzSlow   = kfVz;
                    slowSet  = 1;
                }
                /* a range sample every ~33 ms */
                biasSlow += (kfBias - biasSlow) * 0.033f / (ALT_EST_SLOW_BIAS_TAU_S + 0.033f);
                vzSlow   += (kfVz   - vzSlow)   * 0.033f / (ALT_EST_SLOW_VZ_TAU_S   + 0.033f);
                zRanged   = AltKF_Get(&altKF, ALT_KF_Z);
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
        if(coasting){
            alt->altitude      = zCoast;
            alt->verticalSpeed = vzCoast;
            alt->accelBias     = biasSlow;
        }
        alt->baroBias      = initialized ? AltKF_Get(&altKF, ALT_KF_BARO_BIAS)  : 0.0f;
        alt->accelUp       = accelUp;
        alt->liftVz        = liftVz;
        uint8_t aglSeen    = (rangeSeenAgo < ALT_EST_LOSS_CYCLES);
        uint8_t stalled    = coasting && rangeGone < ALT_EST_STALLED_CYCLES;
        alt->aboveFloor    = aglSeen ? agl : alt->altitude - (rangeAnchored ? rangeOffset : 0.0f);
        if(alt->aboveFloor < 0.0f) alt->aboveFloor = 0.0f;
        alt->agl           = (aglSeen || !stalled) ? agl : alt->aboveFloor;
        alt->aglValid      = aglSeen || stalled;
        alt->rangeStalled  = stalled;
        alt->onFloor       = onFloor;
        alt->valid         = initialized && (lostCycles < ALT_EST_LOSS_CYCLES);
        alt->baroValid     = baroFresh;
        alt->rangeValid    = (rangeUsedAgo < ALT_EST_LOSS_CYCLES);
        alt->dt            = dt;
        alt->timestamp_us  = attitude.timestamp_us;

        AltitudeTopic_Publish(alt);
    }
}