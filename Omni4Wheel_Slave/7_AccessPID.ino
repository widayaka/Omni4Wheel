void SetPIDMinMax(float min, float max){
  motor_pid_min = min; motor_pid_max = max;
}

void SetPIDMotor(float kp, float ki, float kd){
  motor_p = kp; motor_i = ki; motor_d = kd;
}

void SetPIDGainYaw(float kp, float ki, float kd){
  yaw_p = kp; yaw_i = ki; yaw_d = kd;
}

void SetPIDGainOdomX(float kp, float ki, float kd){
  KP_xAxis = kp; KI_xAxis = ki; KD_xAxis = kd;
}

void SetPIDGainOdomY(float kp, float ki, float kd){
  KP_yAxis = kp; KI_yAxis = ki; KD_yAxis = kd;
}

void SetPIDGainOdomTheta(float kp, float ki, float kd){
  KP_theta = kp; KI_theta = ki; KD_theta = kd;
}

void SetPIDGainOdomRobot(float kp, float ki, float kd){
  KP_odom = kp; KI_odom = ki; KD_odom = kd;
}

void SetPointRPM(int speed1, int speed2, int speed3, int speed4){
  setPoint_velocity[0] = speed1; setPoint_velocity[1] = speed2; setPoint_velocity[2] = speed3; setPoint_velocity[3] = speed4;
}

void SetMotorRPM(int speed1, int speed2, int speed3, int speed4){
  motor1(speed1); motor2(speed2); motor3(speed3); motor4(speed4);
}
