#ifndef INC_ACTUATOR_MOTOR_OUTPUT_H_
#define INC_ACTUATOR_MOTOR_OUTPUT_H_

#define MOTOR_COUNT   4

void MotorOutput_Init(void);
void MotorOutput_Update(float roll, float pitch, float yaw, float throttle);

#endif /* INC_ACTUATOR_MOTOR_OUTPUT_H_ */
