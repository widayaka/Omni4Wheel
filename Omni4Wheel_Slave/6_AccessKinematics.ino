void drive(float speed_global_x, float speed_global_y, float speed_angular_w, float offset_heading, int number_of_wheels, float wheelRadius, float robotRadius, float *motorOut){
  float del_angle = 360.0f / number_of_wheels;
  
  for(int i = 0; i < number_of_wheels; i++){
    float angleDeg = del_angle * i + offset_heading;
    float angleRad = angleDeg * PI / 180.0f;
    
    motorOut[i] = (-sinf(angleRad) * speed_global_x) / R_WHEEL;
    motorOut[i] += (cosf(angleRad) * speed_global_y) / R_WHEEL;
    motorOut[i] += (R_ROBOT * speed_angular_w) / R_WHEEL;
  }
}

void robotOodometry(float MI[2][4], float T[4], float angle, float output[2]) {
    float rad = angle * PI / 180.0f; 
    
    float R[2][2] = {
        {cosf(rad), -sinf(rad)},
        {sinf(rad),  cosf(rad)}
    };

    float M[2][1] = {0};
    
    for(int i = 0; i < 2; i++) {
        M[i][0] = 0;
        for(int j = 0; j < 4; j++) {
            M[i][0] += MI[i][j] * T[j];
        }
    }
    
    for(int i = 0; i < 2; i++) {
        output[i] = 0;
        for(int j = 0; j < 2; j++) {
            output[i] += R[i][j] * M[j][0];
            output_[i] = output[i];
        }
    }
}

//void setRobotPosition(float setPointPosX, float setPointPosY, float setPointPosTheta, float velocity){
//  errorPosXAxis = setPointPosX - RobotActualPositionX;
//  errorPosYAxis = setPointPosY - RobotActualPositionY;
//  errorPosTheta = setPointPosTheta - RobotActualPositionTheta;
//  
//  if (errorPosTheta > 180){errorPosTheta = errorPosTheta - 360;}
//  else if (errorPosTheta < -180){errorPosTheta = errorPosTheta + 360;}
//  
//  TotalDistanceTravelledByRobot = sqrt((errorPosXAxis*errorPosXAxis) + (errorPosYAxis*errorPosYAxis));
//  float heading = atan2(errorPosYAxis,errorPosXAxis);
//
//  P_xAxis = KP_xAxis * TotalDistanceTravelledByRobot;
////  P_xAxis = KP_xAxis * errorPosXAxis;
//  I_xAxis = I_xAxis + errorPosXAxis * KI_xAxis;
//  D_xAxis = (errorPosXAxis - lastErrorPosXAxis) * KD_xAxis;
//  PID_xAxis = P_xAxis + I_xAxis + D_xAxis;
//  
//  P_yAxis = KP_yAxis * TotalDistanceTravelledByRobot;
////  P_yAxis = KP_yAxis * errorPosYAxis;
//  I_yAxis = I_yAxis + errorPosYAxis * KI_yAxis;
//  D_yAxis = (errorPosYAxis - lastErrorPosYAxis) * KD_yAxis;
//  PID_yAxis = P_yAxis + I_yAxis + D_yAxis;
//
//  P_theta = KP_theta * errorPosTheta;
//  I_theta = I_theta + errorPosTheta * KI_theta;
//  D_theta = (errorPosTheta - lastErrorPosTheta) * KD_theta;
//  PID_theta = P_theta + I_theta + D_theta;
//
//  VelocityRobotX = PID_xAxis * errorPosXAxis / TotalDistanceTravelledByRobot;
//  VelocityRobotY = PID_yAxis * errorPosYAxis / TotalDistanceTravelledByRobot;
//  VelocityRobotZ = PID_theta;
//    
//  if (VelocityRobotX > velocity) VelocityRobotX = velocity;
//  if (VelocityRobotY > velocity) VelocityRobotY = velocity;
//  if (VelocityRobotZ > velocity) VelocityRobotZ = velocity;
//
//  lastErrorPosXAxis = errorPosXAxis;
//  lastErrorPosYAxis = errorPosYAxis;
//  lastErrorPosTheta = errorPosTheta;
//
//  if (I_xAxis > 255) I_xAxis = 255;
//  if (I_xAxis < -255) I_xAxis = -255;
//  if (errorPosXAxis == 0) I_xAxis = 0;
//  if (PID_xAxis > motor_pid_max) PID_xAxis = motor_pid_max;
//  if (PID_xAxis < motor_pid_min) PID_xAxis = motor_pid_min;
//  if (PID_xAxis > velocity) PID_xAxis = velocity;
//  if (PID_xAxis < -velocity) PID_xAxis = -velocity;
//
//  if (I_yAxis > 255) I_yAxis = 255;
//  if (I_yAxis < -255) I_yAxis = -255;
//  if (errorPosYAxis == 0) I_yAxis = 0;
//  if (PID_yAxis > motor_pid_max) PID_yAxis = motor_pid_max;
//  if (PID_yAxis < motor_pid_min) PID_yAxis = motor_pid_min;
//  if (PID_yAxis > velocity) PID_yAxis = velocity;
//  if (PID_yAxis < -velocity) PID_yAxis = -velocity;
//
//  if (I_theta > 255) I_theta = 255;
//  if (I_theta < -255) I_theta = -255;
//  if (errorPosYAxis == 0) I_theta = 0;
//  if (PID_theta > motor_pid_max) PID_theta = motor_pid_max;
//  if (PID_theta < motor_pid_min) PID_theta = motor_pid_min;
//  if (PID_theta > velocity) PID_theta = velocity;
//  if (PID_theta < -velocity) PID_theta = -velocity;
//
////  InverseKinematicsNoPID(VelocityRobotX, VelocityRobotY, VelocityRobotZ, velocity, OFFSET_HEADING, 4, R_WHEEL, R_ROBOT);
//
//  Serial.print("SPX:"); Serial.print("\t");
//  Serial.print(setPointPosX); Serial.print("\t");
//  Serial.print("ACX:"); Serial.print("\t");
//  Serial.print(RobotActualPositionX); Serial.print("\t");
//  Serial.print("ERX:"); Serial.print("\t");
//  Serial.print(errorPosXAxis); Serial.print("\t");
//  Serial.print("PIDX:"); Serial.print("\t");
//  Serial.print(PID_xAxis); Serial.print("\t");
//
//  Serial.print("SPY:"); Serial.print("\t");
//  Serial.print(setPointPosY); Serial.print("\t");
//  Serial.print("ACY:"); Serial.print("\t");
//  Serial.print(RobotActualPositionY); Serial.print("\t");
//  Serial.print("ERY:"); Serial.print("\t");
//  Serial.print(errorPosYAxis); Serial.print("\t");
//  Serial.print("PIDX:"); Serial.print("\t");
//  Serial.print(PID_yAxis); Serial.print("\t");
//
//  Serial.print("SPT:"); Serial.print("\t");
//  Serial.print(setPointPosTheta); Serial.print("\t");
//  Serial.print("ACT:"); Serial.print("\t");
//  Serial.print(RobotActualPositionTheta); Serial.print("\t");
//  Serial.print("ERT:"); Serial.print("\t");
//  Serial.print(errorPosTheta); Serial.print("\t");
//  Serial.print("PIDT:"); Serial.print("\t");
//  Serial.print(PID_theta); Serial.print("\t");
//  Serial.println();
//  delay(20);
//}

void setRobotPosition(float x, float y, float w, float maxSpeedLin, float maxSpeedAng) {
  RobotSetPointX = x;
  RobotSetPointY = y;
  RobotSetPointTheta = w;
  maxSpeedRobotLin = maxSpeedLin;
  maxSpeedRobotAng = maxSpeedAng;
}

void setRobotSpeed(float x, float y, float w) {
  drive(x, y, w, OFFSET_HEADING, NUM_OF_MOTORS, R_WHEEL, R_ROBOT, setPoint_motor);
}
