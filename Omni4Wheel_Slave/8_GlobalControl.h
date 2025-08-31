//extern float setPointMotor[NUM_OF_MOTORS];
void checkSetPointMotor();
void IRAM_ATTR globalMotorControl();
void IRAM_ATTR globalPositionControl();

extern bool enableMotorControl;
extern bool enablePositionControl;
