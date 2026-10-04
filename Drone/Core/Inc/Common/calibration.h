#ifndef INC_COMMON_CALIBRATION_H_
#define INC_COMMON_CALIBRATION_H_

#include <stdint.h>

/* ==========================================================
 * 					SENSOR CALIBRATION
 * ==========================================================
 * Done at boot, one sensor after the other:
 *   1. BMI088  gyro + accel bias   keep the drone still and level   ~20 s
 *   2. BMP388  ground pressure     keep it on the ground            ~2 s
 *   3. QMC5883 hard-iron offsets   turn it through every orientation 30 s
 *
 * 1 = calibrate again at this boot.
 * 0 = use the stored values below, no waiting.
 * The debug task prints every result in the format below: paste it here and
 * set the flag to 0.
 * ========================================================== */

#define CALIB_BMI088_RUN			1
#define CALIB_BMP388_RUN			1
#define CALIB_QMC5883_RUN			0

/* BMI088 (FRD body frame): gyro rad/s, accel m/s^2 */
#define CALIB_BMI088_GYRO_BIAS_X	0.000112f
#define CALIB_BMI088_GYRO_BIAS_Y	-0.000351f
#define CALIB_BMI088_GYRO_BIAS_Z	-0.000408f
#define CALIB_BMI088_ACCEL_BIAS_X	0.117525f
#define CALIB_BMI088_ACCEL_BIAS_Y	0.626700f
#define CALIB_BMI088_ACCEL_BIAS_Z	0.064246f

/* BMP388: pressure (Pa) that reads as altitude 0. The weather moves it by
 * several hPa from day to day (1 hPa ~ 8 m), so a stored value makes the
 * altitude read an offset on the ground; it only takes 2 s to measure. */
#define CALIB_BMP388_GROUND_PA		101099.8f

/* QMC5883: hard-iron offsets, raw LSB */
#define CALIB_QMC5883_OFFSET_X		107.0f
#define CALIB_QMC5883_OFFSET_Y		90.0f
#define CALIB_QMC5883_OFFSET_Z		-285.0f

#define CALIB_QMC5883_DURATION_MS	30000U

typedef enum{
	CALIB_SENSOR_BMI088 = 0,
	CALIB_SENSOR_BMP388,
	CALIB_SENSOR_QMC5883,
	CALIB_SENSOR_COUNT
} Calib_Sensor_t;

typedef enum{
	CALIB_WAITING = 0,		/* an earlier sensor is still calibrating */
	CALIB_RUNNING,
	CALIB_DONE,				/* measured at this boot */
	CALIB_STORED,			/* stored values loaded */
	CALIB_FAILED,			/* measurement rejected, sensor not used */
	CALIB_NO_SENSOR,		/* sensor did not answer at init */
} Calib_Status_t;

void Calib_Start(Calib_Sensor_t s);
void Calib_SetStatus(Calib_Sensor_t s, Calib_Status_t status);
Calib_Status_t Calib_GetStatus(Calib_Sensor_t s);
uint32_t Calib_ElapsedMs(Calib_Sensor_t s);
uint8_t Calib_Attempts(Calib_Sensor_t s);
uint8_t Calib_IsTurn(Calib_Sensor_t s);
uint8_t Calib_Finished(Calib_Sensor_t s);
uint8_t Calib_Usable(Calib_Sensor_t s);
uint8_t Calib_ReadyToArm(void);
const char *Calib_StatusName(Calib_Status_t status);

#endif /* INC_COMMON_CALIBRATION_H_ */
