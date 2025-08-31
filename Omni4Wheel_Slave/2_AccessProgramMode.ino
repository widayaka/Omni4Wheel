void RobotHoldPosition(){
  SerialDataReceived(); SerialDataParsing(); SerialDataConversion(); SerialDataReset();
  display.clearDisplay(); display.setTextSize(1); display.setTextColor(SSD1306_WHITE);

  float DataYaw = SerialData[1];
  float DataSetPoint = SerialData[2];
  
  errorYaw = DataYaw - DataSetPoint;

  if (errorYaw > 180){errorYaw = errorYaw - 360;}
  else if (errorYaw < -180){errorYaw = errorYaw + 360;}
  
  P_yaw = errorYaw * yaw_p;
  I_yaw += errorYaw * yaw_i;
  D_yaw = (errorYaw - lastErrorYaw) * yaw_d;
  PID_yaw = P_yaw + I_yaw + D_yaw;

  if (PID_yaw > 255)  PID_yaw = 255;
  if (PID_yaw < -255) PID_yaw = -255;

  if (I_yaw > 255) I_yaw = 255;
  if (I_yaw < -255) I_yaw = -255;

  if (errorYaw == 0) I_yaw = 0;

  lastErrorYaw = errorYaw;

  display.setCursor(0,0);   display.print("Yaw");
  display.setCursor(20,0);  display.print(":");
  display.setCursor(25,0);  display.print(DataYaw);

  display.setCursor(70,0);  display.print("SP");
  display.setCursor(85,0);  display.print(":");
  display.setCursor(90,0);  display.print(DataSetPoint);

  display.display();

  DriveMotor(PID_yaw, PID_yaw, PID_yaw, PID_yaw);
  if (SerialData[0] == 0 || SerialData[0] == 1) menu = 0;
}

void RobotJoystickControl(){
  SerialDataReceived(); SerialDataParsing(); SerialDataConversion(); SerialDataReset();
  InverseKinematicsNoPID(SerialData[1], SerialData[2], SerialData[3], 255, 45, 4, R_WHEEL, R_ROBOT);
  if (SerialData[0] == 0 || SerialData[0] == 1) menu = 0;
}

void RobotOdometry(){
  SerialDataReceived(); SerialDataParsing(); SerialDataConversionOdometry(); 
  display.clearDisplay(); display.setTextSize(1); display.setTextColor(SSD1306_WHITE);

  yaw_robot = SerialData[1];
  yaw_polar = fmodf((360.0 - yaw_robot),360);
  RobotActualPositionTheta = yaw_polar;
  setPointYaw = 0.0;

//  display.setCursor(0,0);   display.print("Odometry Mode");
//  display.setCursor(0,10);  display.print("Orientation");
//  display.setCursor(30,10); display.print(":");
//  display.setCursor(70,10); display.print(yaw_polar);
//  Serial.print(RobotActualPositionTheta); Serial.print("\t");
//
//  Serial.print(RobotActualPositionX); Serial.print("\t");
//  Serial.print(RobotActualPositionY); Serial.print("\t");
//  Serial.println();
  
  display.display();
  
  if (flag_20ms){flag_20ms = false; odometryTimerLoop();}
  setRobotPosition(2, 4, setPointYaw, 100);

  if (SerialData[0] == 0 || SerialData[0] == 1) menu = 0;
  SerialDataReset();
}

void RobotObjectTracking(){
  SerialDataReceived(); 
  SerialDataParsing(); 
  SerialDataConversion(); 
  SerialDataReset();

  if (SerialData[0] == 0 || SerialData[0] == 1) menu = 0;
}

void RobotROS2Navigation(){
  SerialDataReceived(); 
  SerialDataParsing(); 
  SerialDataConversion(); 
  SerialDataReset();

  if (SerialData[0] == 0 || SerialData[0] == 1) menu = 0;
}
