#ifndef __ACCESS_MOTOR__
#define __ACCESS_MOTOR__

void IRAM_ATTR motor1(int PWM);
void IRAM_ATTR motor2(int PWM);
void IRAM_ATTR motor3(int PWM);
void IRAM_ATTR motor4(int PWM);
void IRAM_ATTR DriveMotor(int speedVal1, int speedVal2, int speedVal3, int speedVal4);

#endif
