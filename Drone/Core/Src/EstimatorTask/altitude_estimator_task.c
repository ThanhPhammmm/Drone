#include "altitude_estimator_task.h"
#include "attitude_topic.h"
#include "baro_topic.h"
#include "altitude_topic.h"
#include "Const.h"

#define ALT_EST_OMEGA               1.0f
#define ALT_EST_K1                  (3.0f * ALT_EST_OMEGA)
#define ALT_EST_K2                  (3.0f * ALT_EST_OMEGA * ALT_EST_OMEGA)
#define ALT_EST_K3                  (ALT_EST_OMEGA * ALT_EST_OMEGA * ALT_EST_OMEGA)

#define ALT_EST_BARO_MAX_AGE_US     200000U
#define ALT_EST_ACCEL_BIAS_LIMIT    2.0f      /* m/s^2 */
#define ALT_EST_DT_MAX              0.01f

AltitudeEstimator_Handle_t altitudeEstimator;

void AltitudeEstimator_SetTaskHandle(TaskHandle_t handle){
	altitudeEstimator.altitudeTask = handle;
}

void AltitudeEstimatorTask(void *argument){
	AltitudeEstimator_SetTaskHandle(xTaskGetCurrentTaskHandle());
	AttitudeTopic_Subscribe(altitudeEstimator.altitudeTask, ALTITUDE_ATTITUDE_ID_TASK);

    Attitude_Data_t attitude;
    Baro_Data_t     baro;

    float z = 0.0f, vz = 0.0f, accelBias = 0.0f;
    uint8_t initialized = 0;

    while(1){
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if(AttitudeTopic_Copy(&attitude) != pdPASS) continue;

        float dt = attitude.dt;
        if(dt <= 0.0f || dt > ALT_EST_DT_MAX) continue;

        uint8_t baroFresh = 0;
        if(BaroTopic_Copy(&baro, 0) == pdPASS && baro.timestamp_us != 0){
            if((uint32_t)(attitude.timestamp_us - baro.timestamp_us) < ALT_EST_BARO_MAX_AGE_US){
                baroFresh = 1;
            }
        }

        if(!initialized){
            if(!baroFresh) continue;
            z         = baro.altitude_m;
            vz        = 0.0f;
            accelBias = 0.0f;
            initialized = 1;
        }
        float az = attitude.accelUp - accelBias;

        if(baroFresh){
            // observer error
            // handle drift accelerometer
            float err = baro.altitude_m - z;

            z         += (vz + ALT_EST_K1 * err) * dt;
            vz        += (az + ALT_EST_K2 * err) * dt;
            accelBias += (-ALT_EST_K3 * err) * dt;

            if(accelBias >  ALT_EST_ACCEL_BIAS_LIMIT) accelBias =  ALT_EST_ACCEL_BIAS_LIMIT;
            if(accelBias < -ALT_EST_ACCEL_BIAS_LIMIT) accelBias = -ALT_EST_ACCEL_BIAS_LIMIT;
        }
        else{
            z  += vz * dt;
            vz += az * dt;
        }

        Altitude_Data_t *alt = &altitudeEstimator.data;
        alt->altitude      = z;
        alt->verticalSpeed = vz;
        alt->accelUp       = attitude.accelUp;
        alt->accelBias     = accelBias;
        alt->valid         = initialized;
        alt->baroValid     = baroFresh;
        alt->dt            = dt;
        alt->timestamp_us  = attitude.timestamp_us;

        AltitudeTopic_Publish(alt);
    }
}
