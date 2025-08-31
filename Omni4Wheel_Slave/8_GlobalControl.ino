#include "9_SerialPrintWrapper.h"

float setPointMotor[NUM_OF_MOTORS];
float motor_p;
float motor_i;
float motor_d;

#define TH_GOAL_REACHED 0.1
float spXPos = 0;
float spYPos = 0;
float spThPos = 0;
float maxSpeedRobotLin = 0;
float maxSpeedRobotAng = 0;
bool statusGoalReached = false;
float KpCP = 100;

bool enableMotorControl = true;
bool enablePositionControl = true;

void setPIDMotor(float kp, float ki, float kd){
  motor_p = kp;
  motor_i = ki;
  motor_d = kd;
}

void setRobotPosition(float x, float y, float w, float maxSpeedLin, float maxSpeedAng) {
  spXPos = x;
  spYPos = y;
  spThPos = w;
  maxSpeedRobotLin = maxSpeedLin;
  maxSpeedRobotAng = maxSpeedAng;
}

void checkSetPointMotor(){
  SerialPrint("%.1f %.1f %.1f %.1f", setPointMotor[0], setPointMotor[1], setPointMotor[2], setPointMotor[3]);
//  Serial.println(setPointMotor[0]);
}

void setRobotSpeed(float x, float y, float w) {
  drive(x, y, w, OFFSET_HEADING, NUM_OF_MOTORS, R_WHEEL, R_ROBOT, setPointMotor);
}

//void setMotorRPM(float motor[NUM_OF_MOTORS]) {
//  memcpy(setPointMotor, motor, sizeof(motor));
//}

void setMotorRPM(float m1, float m2, float m3, float m4) {
  Serial.println("Setting Motor RPM");
  setPointMotor[0] = m1;
  setPointMotor[1] = m2;
  setPointMotor[2] = m3;
  setPointMotor[3] = m4;
  checkSetPointMotor();
}

void globalMotorControl() {
  if (!enableMotorControl) return;

  static float error[NUM_OF_MOTORS] = {0};
  static float lastError[NUM_OF_MOTORS] = {0};
  float del_angle = 360.0f / NUM_OF_MOTORS;

  for (int i = 0; i < NUM_OF_MOTORS; i++) {
    error[i] = (setPointMotor[i] - encoder_velocity[i]);
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
  if(!enablePositionControl) return;
  
  errorPosXAxis = spXPos - RobotActualPositionX;
  errorPosYAxis = spYPos - RobotActualPositionY;
  errorPosTheta = spThPos - RobotActualPositionTheta;

  if (errorPosTheta > 180) {
    errorPosTheta = errorPosTheta - 360;
  } else if (errorPosTheta < -180) {
    errorPosTheta = errorPosTheta + 360;
  }

  float deltaD = sqrt((errorPosXAxis * errorPosXAxis) + (errorPosYAxis * errorPosYAxis));
  float heading = atan2(errorPosYAxis, errorPosXAxis);

  float P = deltaD * KpCP;
  float PID_V = P;

  if (PID_V > maxSpeedRobotLin) PID_V = maxSpeedRobotLin;

  P_theta = KP_theta * errorPosTheta;
  I_theta = I_theta + errorPosTheta * KI_theta;
  D_theta = (errorPosTheta - lastErrorPosTheta) * KD_theta;
  PID_theta = P_theta + I_theta + D_theta;
  if(PID_theta > maxSpeedRobotAng) PID_theta = maxSpeedRobotAng;
  if(PID_theta < -maxSpeedRobotAng) PID_theta = -maxSpeedRobotAng;

  VelocityRobotX = PID_V * errorPosXAxis / deltaD;
  VelocityRobotY = PID_V * errorPosYAxis / deltaD;
  VelocityRobotZ = 0; // nanti diganti dengan PID_

//  if (VelocityRobotX > velocity) VelocityRobotX = velocity;
//  if (VelocityRobotY > velocity) VelocityRobotY = velocity;
//  if (VelocityRobotZ > velocity) VelocityRobotZ = velocity;

  setRobotSpeed(VelocityRobotX, VelocityRobotY, VelocityRobotZ);
}
