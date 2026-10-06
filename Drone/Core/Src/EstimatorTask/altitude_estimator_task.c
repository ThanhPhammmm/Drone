#include "altitude_estimator_task.h"

#define ALT_EST_OMEGA               2.0f
#define ALT_EST_K1                  (3.0f * ALT_EST_OMEGA)
#define ALT_EST_K2                  (3.0f * ALT_EST_OMEGA * ALT_EST_OMEGA)
#define ALT_EST_K3                  (ALT_EST_OMEGA * ALT_EST_OMEGA * ALT_EST_OMEGA)

#define ALT_EST_BARO_MAX_AGE_US     30000
#define ALT_EST_ACCEL_BIAS_LIMIT    2.0f      /* m/s^2 */

#define ALT_EST_PERIOD_S            0.01f     /* run the observer on every 10 ms of samples */
#define ALT_EST_GAP_MAX_S           0.05f     /* longer gap between samples: start the window over */
#define ALT_EST_BARO_LOSS_CYCLES    50U

#define LIFT_BASE_TAU_S             0.5f      /* s, standing reading, learned while the motors do not fly */
#define LIFT_LEAK_TAU_S             1.0f      /* s, forgets a small accelerometer error instead of adding it up */

AltitudeEstimator_Handle_t altitudeEstimator;

void AltitudeEstimator_SetTaskHandle(TaskHandle_t handle){
	altitudeEstimator.altitudeTask = handle;
}

void AltitudeEstimatorTask(void *argument){
	AltitudeEstimator_SetTaskHandle(xTaskGetCurrentTaskHandle());
	AttitudeTopic_Subscribe(altitudeEstimator.altitudeTask, ALTITUDE_ESTIMATOR_ID_TASK);

    Attitude_Data_t attitude;
    Baro_Data_t     baro;

    float z = 0.0f, vz = 0.0f, accelBias = 0.0f;
    uint8_t initialized = 0;
    uint16_t baroLostCycles = 0;

    uint32_t lastSampleUs = 0;
    uint8_t  haveSample   = 0;
    float    dtAcc        = 0.0f;
    float    accelUpAcc   = 0.0f;

    float    liftBase     = 0.0f;   /* m/s^2, accelUp while standing */
    uint8_t  liftBaseSet  = 0;
    float    liftVz       = 0.0f;   /* m/s, accelerometer-only lift-off speed */

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
            liftVz    = 0.0f;
        }
        else if(!Arm_IsAirborne()){
            liftVz += (accelUp - liftBase) * dt;
            
            /* If the aircraft only vibrates on the ground without taking off, 
            integration errors can accumulate over time and falsely indicate 
            that it is flying. A leak continuously reduces the accumulated 
            value toward zero, cancelling out mechanical vibration effects.*/
            liftVz -= liftVz * dt / LIFT_LEAK_TAU_S;
        }
        else{
            liftVz = 0.0f;
        }

        uint8_t baroFresh = 0;
        if(BaroTopic_Copy(&baro, 0) == pdPASS && baro.timestamp_us != 0){
            int32_t age = Time_DiffUs(attitude.timestamp_us, baro.timestamp_us);
            if(age < ALT_EST_BARO_MAX_AGE_US && age > -ALT_EST_BARO_MAX_AGE_US){
                baroFresh = 1;
            }
        }

        if(baroFresh) baroLostCycles = 0;
        else if(baroLostCycles < ALT_EST_BARO_LOSS_CYCLES) baroLostCycles++;

        if(!initialized && baroFresh){
            z         = baro.altitude_m;
            vz        = 0.0f;
            accelBias = 0.0f;
            initialized = 1;
        }

        if(initialized){
            float az = accelUp - accelBias;

            if(baroFresh){
                // observer error
                // handle drift accelerometer
                float err = baro.altitude_m - z;

                z         += (vz + ALT_EST_K1 * err) * dt;
                vz        += (az + ALT_EST_K2 * err) * dt;
                accelBias += (-ALT_EST_K3 * err) * dt;
            }
            else{
                z  += vz * dt;
                vz += az * dt;
            }

            if(!Arm_IsAirborne()){
                vz        = liftVz;
                accelBias = liftBase;
            }

            if(accelBias >  ALT_EST_ACCEL_BIAS_LIMIT) accelBias =  ALT_EST_ACCEL_BIAS_LIMIT;
            if(accelBias < -ALT_EST_ACCEL_BIAS_LIMIT) accelBias = -ALT_EST_ACCEL_BIAS_LIMIT;
        }

        Altitude_Data_t *alt = &altitudeEstimator.data;
        alt->altitude      = z;
        alt->verticalSpeed = vz;
        alt->accelUp       = accelUp;
        alt->accelBias     = accelBias;
        alt->liftVz        = liftVz;
        alt->valid         = initialized && (baroLostCycles < ALT_EST_BARO_LOSS_CYCLES);
        alt->baroValid     = baroFresh;
        alt->dt            = dt;
        alt->timestamp_us  = attitude.timestamp_us;

        AltitudeTopic_Publish(alt);
    }
}
