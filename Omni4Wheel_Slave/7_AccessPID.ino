void SetPIDMinMax(float min, float max){
  motor_pid_min = min;
  motor_pid_max = max;
}

void SetPIDGainRPM(float kp, float ki, float kd){
  motor_p = kp;
  motor_i = ki;
  motor_d = kd;
}

void SetPIDGainYaw(float kp, float ki, float kd){
  yaw_p = kp;
  yaw_i = ki;
  yaw_d = kd;
}

void SetPIDGainOdomX(float kp, float ki, float kd){
  KP_xAxis = kp; 
  KI_xAxis = ki; 
  KD_xAxis = kd;
}

void SetPIDGainOdomY(float kp, float ki, float kd){
  KP_yAxis = kp; 
  KI_yAxis = ki; 
  KD_yAxis = kd;
}

void SetPIDGainOdomTheta(float kp, float ki, float kd){
  KP_theta = kp;
  KI_theta = ki;
  KD_theta = kd;
}

void SetPointRPM(int speed1, int speed2, int speed3, int speed4){
  setPoint_velocity_monitor[0] = speed1;
  setPoint_velocity_monitor[1] = speed1;
  setPoint_velocity_monitor[2] = speed1;
  setPoint_velocity_monitor[3] = speed1;

  setPoint_velocity[0] = speed1;
  setPoint_velocity[1] = speed2;
  setPoint_velocity[2] = speed3;
  setPoint_velocity[3] = speed4;
}

void SetMotorRPM(int speed1, int speed2, int speed3, int speed4){
  motor1(speed1);
  motor2(speed2);
  motor3(speed3);
  motor4(speed4);
}

void MotorRPMWithPID(){
  encoderAll_RPM();
  for (int i = 0; i < NUM_OF_MOTORS; i++){
    encoderAll_RPM();
    
    error[i] = (setPoint_velocity[i] - encoder_velocity[i]);
    
    P[i] = (float)(error[i] * motor_p);
    I[i] += error[i] * motor_i;
    D[i] = (error[i] - lastError[i]) * motor_d;
    PIDForRPM[i] = P[i] + I[i] + D[i];

    if (PIDForRPM[i] > motor_pid_max) PIDForRPM[i] = motor_pid_max;
    if (PIDForRPM[i] < motor_pid_min) PIDForRPM[i] = motor_pid_min;

    if (I[i] > motor_pid_max) I[i] = motor_pid_max;
    if (I[i] < motor_pid_min) I[i] = motor_pid_min;

    lastError[i] = error[i];
  }

  SetMotorRPM(PIDForRPM[0], PIDForRPM[1], PIDForRPM[2], PIDForRPM[3]);

//  display.clearDisplay(); display.setTextSize(1); display.setTextColor(SSD1306_WHITE);
//
//  display.setCursor(0,0); display.println(setPoint_velocity[0]);
//  display.setCursor(0,10); display.println(encoder_velocity[0]);
//
//  display.setCursor(30,0); display.println(setPoint_velocity_monitor[1]);
//  display.setCursor(30,10); display.println(encoder_velocity[1] / ENCODER_PPR * GEAR_RATIO);
//
//  display.setCursor(60,0); display.println(setPoint_velocity_monitor[2]);
//  display.setCursor(60,10); display.println(encoder_velocity[2] / ENCODER_PPR * GEAR_RATIO);
//
//  display.setCursor(90,0); display.println(setPoint_velocity_monitor[3]);
//  display.setCursor(90,10); display.println(encoder_velocity[3] / ENCODER_PPR * GEAR_RATIO);
//  
//  display.display(); 
}
