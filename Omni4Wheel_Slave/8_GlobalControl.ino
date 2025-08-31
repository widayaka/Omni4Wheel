int setPointMotor[NUM_OF_MOTORS];

#define TH_GOAL_REACHED 0.1
float spXPos = 0; 
float spYPos = 0;
float spThPos = 0;
bool statusGoalReached = false;

void setRobotSpeed(float);

void setMotorRPM(int motor[NUM_OF_MOTOR]) {
  memcpy(setPoinMotor, motor, sizeof(motor));
}

void globalMotorControl(){
  static float error[NUM_OF_MOTORS] = {0};
  static float lastError[NUM_OF_MOTORS] = {0};
  float del_angle = 360.0f / NUM_OF_MOTORS;
  float motor[number_of_wheels];
  
  encoderAll_RPM();
  
  for(int i = 0; i < number_of_wheels; i++){
    float angleDeg = (del_angle * i) + offset_heading;
    float angleRad = angleDeg * PI / 180.0f;
    
    encoderAll_RPM();
    motor[i] = (-speed_global_x * sinf(angleRad)) / wheelRadius;
    motor[i] += (speed_global_y * cosf(angleRad)) / wheelRadius;
    motor[i] += (speed_angular_w * robotRadius) / wheelRadius;
      
    if (motor[i] > maxSpeed)  motor[i] = maxSpeed;
    if (motor[i] < -maxSpeed) motor[i] = -maxSpeed;

    error[i] = (motor[i] - encoder_velocity[i]);
    P[i] = error[i] * motor_p;
    I[i] += error[i] * motor_i;
    D[i] = (error[i] - lastError[i]) * motor_d;
    PIDForRPM[i] = P[i] + I[i] + D[i];

    if (PIDForRPM[i] > motor_pid_max) PIDForRPM[i] = motor_pid_max;
    if (PIDForRPM[i] < motor_pid_min) PIDForRPM[i] = motor_pid_min;

    if (I[i] > motor_pid_max) I[i] = motor_pid_max;
    if (I[i] < motor_pid_min) I[i] = motor_pid_min;

    if (error[i] == 0) I[i] = 0;
    
    lastError[i] = error[i];
  }
  
  DriveMotor(PIDForRPM[0], PIDForRPM[1], PIDForRPM[2], PIDForRPM[3]);
}

void globalPositionControl() {
  
}
