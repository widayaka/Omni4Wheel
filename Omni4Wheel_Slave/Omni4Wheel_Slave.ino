#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define PB_RUN 12
#define PB_UP 5
#define PB_DOWN 3
#define PB_OK 15

#define PUSH_BUTTON_OK_IS_PRESSED   digitalRead(PB_OK) == LOW
#define PUSH_BUTTON_DOWN_IS_PRESSED digitalRead(PB_DOWN) == LOW
#define PUSH_BUTTON_UP_IS_PRESSED   digitalRead(PB_UP) == LOW
#define PUSH_BUTTON_RUN_IS_PRESSED  digitalRead(PB_RUN) == LOW

#define NUM_OF_DATA     4
#define NUM_OF_MOTORS   4
#define NUM_OF_ENCODER  4
#define ENCODER_PPR     11
#define R_WHEEL         0.03
#define R_ROBOT         0.1

#define OFFSET_HEADING 45

#define PI    3.14159265359
#define CW    0
#define CCW   1

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32

#define OLED_RESET     -1         
#define SCREEN_ADDRESS 0x3C       
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define RX2 16
#define TX2 17
#define SLAVE_SERIAL_BAUDRATE 115200
HardwareSerial SlaveSerial2(2);

volatile int16_t encoder_cnt[NUM_OF_MOTORS];
volatile int16_t encoder_prev_cnt[NUM_OF_MOTORS];
volatile int16_t encoder_last_cnt[NUM_OF_MOTORS];
volatile int16_t encoder_RPM[NUM_OF_MOTORS];

float setPoint_velocity[NUM_OF_MOTORS];
float setPoint_RPM[NUM_OF_MOTORS];
float error[NUM_OF_MOTORS];
float lastError[NUM_OF_MOTORS];

float P[NUM_OF_MOTORS];
float I[NUM_OF_MOTORS];
float D[NUM_OF_MOTORS];
float PIDForRPM[NUM_OF_MOTORS];

float min_rpm = 0;
float max_rpm = 0;

float motor_p = 0;
float motor_i = 0;
float motor_d = 0;

float motor_pid_min;
float motor_pid_max;

float setPointYaw;
float errorYaw;
float lastErrorYaw;
float P_yaw;
float I_yaw;
float D_yaw;
float PID_yaw;

float yaw_p;
float yaw_i;
float yaw_d;

float yaw_pid_min;
float yaw_pid_max;

float P_xAxis; 
float I_xAxis; 
float D_xAxis;
float KP_xAxis; 
float KI_xAxis; 
float KD_xAxis;
float errorPosXAxis;
float lastErrorPosXAxis; 
float PID_xAxis;
      
float P_yAxis; 
float I_yAxis; 
float D_yAxis;
float KP_yAxis; 
float KI_yAxis; 
float KD_yAxis;
float errorPosYAxis;
float lastErrorPosYAxis;
float PID_yAxis;
      
float P_theta; 
float I_theta; 
float D_theta;
float KP_theta; 
float KI_theta; 
float KD_theta;
float errorPosTheta;
float lastErrorPosTheta;
float PID_theta;

float P_odom;
float I_odom;
float D_odom;
float KP_odom;
float KI_odom;
float KD_odom;
float PID_odom;

bool statusWaypointReached = false;

float RobotSetPointX = 0.0;
float RobotSetPointY = 0.0;
float RobotSetPointTheta = 0.0;

float RobotActualPositionX = 0.0;
float RobotActualPositionY = 0.0;
float RobotActualPositionTheta = 0.0;

float maxSpeedRobotLin = 0;
float maxSpeedRobotAng = 0;

float DistanceRobotToTarget = 0.0;
float TotalDistanceTravelledByRobot = 0.0;

float VelocityRobotX = 0.0;
float VelocityRobotY = 0.0;
float VelocityRobotZ = 0.0;

float PID_velocity;

float ErrorPositionX = 0.0;
float ErrorPositionY = 0.0;
float ErrorPositionTheta = 0.0;

float v_robot_l_x = 0.0;
float v_robot_l_y = 0.0;

unsigned long currentTime_odom;
unsigned long previousTime_odom;

float velocity_wheel[4];
float velocity_robot[2];
float output_[2];
float theta_dot[4];
      
float yaw_robot;
float yaw_polar;

int deltaTime_odom;
int delta_encoder[4];

int PIN_MOT_1A = 23;
int PIN_MOT_1B = 19;
int PIN_ENC_1A = 36;
int PIN_ENC_1B = 39;
volatile int VAL_ENC_1A = 0;
volatile int VAL_ENC_1B = 0;

int PIN_MOT_2A = 18;
int PIN_MOT_2B = 4;
int PIN_ENC_2A = 34;
int PIN_ENC_2B = 35;
volatile int VAL_ENC_2A = 0;
volatile int VAL_ENC_2B = 0;

int PIN_MOT_3A = 2;
int PIN_MOT_3B = 13;
int PIN_ENC_3A = 32;
int PIN_ENC_3B = 33;
volatile int VAL_ENC_3A = 0;
volatile int VAL_ENC_3B = 0;

int PIN_MOT_4A = 14;
int PIN_MOT_4B = 27;
int PIN_ENC_4A = 25;
int PIN_ENC_4B = 26;
volatile int VAL_ENC_4A = 0;
volatile int VAL_ENC_4B = 0;

int frequency=5000;
int resolution=8;
int pwmChannel1 = 0; 
int pwmChannel2 = 1; 
int pwmChannel3 = 2; 
int pwmChannel4 = 3;

unsigned long currentTime = 0;
unsigned long previousTime = 0;

int interval = 20;

int menu=0;
int pilih=0;
int pilih_menu_encoder=0; 
int pilih_menu_motor=0;
    
const char startDataIdentifier = '*';
const char dataSeparator = ',';
const char stopDataIdentifier = '#';

int16_t sendDataEncoder1; 
int16_t sendDataEncoder2; 
int16_t sendDataEncoder3;
int16_t sendDataEncoder4;
    
int16_t sendDataRPM1; 
int16_t sendDataRPM2; 
int16_t sendDataRPM3;
int16_t sendDataRPM4;

char dataReceived;

const byte arraySize = 21;

String dataPackage;
String bufferDataIn;
String bufferDataParsing[arraySize];

bool dataIsComplete;

byte indexOfData;

int counter;
float SerialData[10];
float data1;
float data2;
float data3; 
float data4;

int UARTStatusDisplay;

TaskHandle_t Task_ReadencoderAll_RPM = NULL;
void encoderAll_RPM(void *parameter){
  for(;;){
    currentTime = millis();
    if (currentTime - previousTime >= interval){
      previousTime = currentTime;
      for(int i = 0; i < NUM_OF_MOTORS; i++){
        encoder_RPM[i] = ((encoder_cnt[i] - encoder_prev_cnt[i]) * 60) / ENCODER_PPR;
        encoder_prev_cnt[i] = encoder_cnt[i];
      }
    }
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

bool enableMotorControl = true;
TaskHandle_t Task_globalMotorControl = NULL;
void globalMotorControl(void *parameter){
  for(;;){
    if (!enableMotorControl) {
      vTaskDelay(20 / portTICK_PERIOD_MS); // tidur sebentar
      continue;
    }
    
    for (int i = 0; i < NUM_OF_MOTORS; i++){    
      error[i] = (setPoint_RPM[i] - encoder_RPM[i]);
      P[i] = (float)(error[i] * motor_p);
      I[i] += error[i] * motor_i;
      D[i] = (error[i] - lastError[i]) * motor_d;
      PIDForRPM[i] = P[i] + I[i] + D[i];

      if (I[i] > motor_pid_max) I[i] = motor_pid_max;
      if (I[i] < motor_pid_min) I[i] = motor_pid_min;
      
      if (PIDForRPM[i] > motor_pid_max) PIDForRPM[i] = motor_pid_max;
      if (PIDForRPM[i] < motor_pid_min) PIDForRPM[i] = motor_pid_min;

      if (error[i] == 0) I[i] = 0;
      
      lastError[i] = error[i];
    }
    DriveMotor(PIDForRPM[0], PIDForRPM[1], PIDForRPM[2], PIDForRPM[3]);
    vTaskDelay(20 / portTICK_PERIOD_MS); 
  }
}

// Matrix Odometry : N = 4, Heading Offset = 45, Robot Radius = 1.0, Wheel Radius = 0.03
float m_odometry[2][4] = {
    {-0.010607f, -0.010607f, 0.010607f, 0.010607f},
    {0.010607f, -0.010607f, -0.010607f, 0.010607f}
};

TaskHandle_t Task_odometryTimerLoop = NULL;
void odometryTimerLoop(void *parameter){
  for(;;){
    currentTime_odom = millis();
    deltaTime_odom = currentTime_odom - previousTime_odom;
    float dt = deltaTime_odom / 1000.0f;
    
    for (int i = 0; i < NUM_OF_MOTORS; i++){
      delta_encoder[i] = encoder_cnt[i] - encoder_last_cnt[i];
      theta_dot[i] = (delta_encoder[i] / ENCODER_PPR) * 2.0 * PI / dt;
      velocity_wheel[i] = theta_dot[i] * R_WHEEL;
    }
  
    robotOodometry(m_odometry, velocity_wheel, RobotActualPositionTheta, velocity_robot);
  
    v_robot_l_x = velocity_robot[0];
    v_robot_l_y = velocity_robot[1];
    
    RobotActualPositionX += v_robot_l_x * dt;
    RobotActualPositionY += v_robot_l_y * dt;
  
    previousTime_odom = currentTime_odom;
    for (int i = 0; i < NUM_OF_MOTORS; i++){encoder_last_cnt[i] = encoder_cnt[i];}
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

int currentWaypointIndex = 0;
bool enablePositionControl = true;
TaskHandle_t Task_globalPositionControl = NULL;
void globalPositionControl(void *parameter){
  for(;;){
    if (!enablePositionControl) {
      vTaskDelay(20 / portTICK_PERIOD_MS);
      continue;
    }
    
    errorPosXAxis = RobotSetPointX - RobotActualPositionX;
    errorPosYAxis = RobotSetPointY - RobotActualPositionY;
    errorPosTheta = RobotSetPointTheta - RobotActualPositionTheta;

    if      (errorPosTheta > 180)   {errorPosTheta = errorPosTheta - 360;}
    else if (errorPosTheta < -180)  {errorPosTheta = errorPosTheta + 360;}

    TotalDistanceTravelledByRobot = sqrt((errorPosXAxis*errorPosXAxis) + (errorPosYAxis*errorPosYAxis));

    P_xAxis = KP_xAxis * errorPosXAxis;
    I_xAxis += errorPosXAxis * KI_xAxis;
    D_xAxis = (lastErrorPosXAxis - errorPosXAxis) * KD_xAxis;
    PID_xAxis = P_xAxis + I_xAxis + D_xAxis;

    if (PID_xAxis > maxSpeedRobotLin) PID_xAxis = maxSpeedRobotLin;
    if (PID_xAxis < -maxSpeedRobotLin) PID_xAxis = -maxSpeedRobotLin;

    P_yAxis = KP_yAxis * errorPosXAxis;
    I_yAxis += errorPosYAxis * KI_yAxis;
    D_yAxis = (lastErrorPosYAxis - errorPosYAxis) * KD_yAxis;
    PID_yAxis = P_yAxis + I_yAxis + D_yAxis;

    if (PID_yAxis > maxSpeedRobotLin) PID_yAxis = maxSpeedRobotLin;
    if (PID_yAxis < -maxSpeedRobotLin) PID_yAxis = -maxSpeedRobotLin;

    P_theta = KP_theta * errorPosTheta;
    I_theta = I_theta + errorPosTheta * KI_theta;
    D_theta = (errorPosTheta - lastErrorPosTheta) * KD_theta;
    PID_theta = P_theta + I_theta + D_theta;

    if(PID_theta > maxSpeedRobotAng) PID_theta = maxSpeedRobotAng;
    if(PID_theta < -maxSpeedRobotAng) PID_theta = -maxSpeedRobotAng;

    VelocityRobotX = PID_xAxis * errorPosXAxis / TotalDistanceTravelledByRobot;
    VelocityRobotY = PID_yAxis * errorPosYAxis / TotalDistanceTravelledByRobot;
    VelocityRobotZ = 0;
    
    setRobotSpeed(VelocityRobotX, VelocityRobotY, VelocityRobotZ);
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);
  SlaveSerial2.begin(SLAVE_SERIAL_BAUDRATE, SERIAL_8N1, RX2, TX2);
  
  pinMode(PB_RUN, INPUT_PULLUP);
  pinMode(PB_UP, INPUT_PULLUP);
  pinMode(PB_DOWN, INPUT_PULLUP);
  pinMode(PB_OK, INPUT_PULLUP);

  ledcSetup(pwmChannel1, frequency, resolution);
  ledcAttachPin(PIN_MOT_1A, pwmChannel1); 
  pinMode(PIN_MOT_1B, OUTPUT);
  pinMode(PIN_ENC_1A, INPUT_PULLUP);
  pinMode(PIN_ENC_1B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_1A), encoder1_ISR, CHANGE); 
  
  ledcSetup(pwmChannel2, frequency, resolution);
  ledcAttachPin(PIN_MOT_2A, pwmChannel2); 
  pinMode(PIN_MOT_2B, OUTPUT);
  pinMode(PIN_ENC_2A, INPUT_PULLUP);
  pinMode(PIN_ENC_2B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_2A), encoder2_ISR, CHANGE); 

  ledcSetup(pwmChannel3, frequency, resolution);
  ledcAttachPin(PIN_MOT_3A, pwmChannel3); 
  pinMode(PIN_MOT_3B, OUTPUT);
  pinMode(PIN_ENC_3A, INPUT_PULLUP);
  pinMode(PIN_ENC_3B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_3A), encoder3_ISR, CHANGE); 

  ledcSetup(pwmChannel4, frequency, resolution);
  ledcAttachPin(PIN_MOT_4A, pwmChannel4); 
  pinMode(PIN_MOT_4B, OUTPUT);
  pinMode(PIN_ENC_4A, INPUT_PULLUP);
  pinMode(PIN_ENC_4B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_4A), encoder4_ISR, CHANGE); 

  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }

  SetPIDMinMax(-255, 255);
  SetPIDGainYaw(10, 0, 5);
  
  enableMotorControl = true;
  SetPIDMotor(0.5, 0.25, 0);
  SetRPMMinMax(-200, 200);
  
  enablePositionControl = true;
  SetPIDGainOdomX(100, 0, 0.5);
  SetPIDGainOdomY(100, 0, 0.5);
  SetPIDGainOdomRobot(50,0,0);
  
  for(int i = 0; i < NUM_OF_MOTORS; i++){encoder_cnt[i] = 0;}
  previousTime_odom = millis();
  RobotBootScreen();

  xTaskCreatePinnedToCore(
    encoderAll_RPM,               // Task function
    "encoderAll_RPM",             // Task name
    4096,                         // Stack size (bytes)
    NULL,                         // Parameters
    1,                            // Priority
    &Task_ReadencoderAll_RPM,     // Task handle
    0                             // Core 0
  );

  xTaskCreatePinnedToCore(
    globalMotorControl,           // Task function
    "globalMotorControl",         // Task name
    4096,                         // Stack size (bytes)
    NULL,                         // Parameters
    1,                            // Priority
    &Task_globalMotorControl,     // Task handle
    0                             // Core 0
  );

  xTaskCreatePinnedToCore(
    odometryTimerLoop,            // Task function
    "odometryTimerLoop",          // Task name
    4096,                         // Stack size (bytes)
    NULL,                         // Parameters
    1,                            // Priority
    &Task_odometryTimerLoop,      // Task handle
    0                             // Core 0
  );

  xTaskCreatePinnedToCore(
    globalPositionControl,        // Task function
    "globalPositionControl",      // Task name
    4096,                         // Stack size (bytes)
    NULL,                         // Parameters
    1,                            // Priority
    &Task_globalPositionControl,  // Task handle
    0                             // Core 0
  );
}

void loop() {
  // while (menu == 0) {RobotHomeScreen();}
  // while (menu == 1) {RobotMenuEncoder();}
  // while (menu == 2) {RobotMenuMotor();}

  // while (menu == 6) {RobotOdometry();}
  // while (menu == 7) {RobotHoldPosition();}
  // while (menu == 8) {RobotJoystickControl();}
  // while (menu == 9) {RobotOdometry();}
  drive(1, 0, 0, OFFSET_HEADING, NUM_OF_MOTORS, R_WHEEL, R_ROBOT, setPoint_RPM);
  Serial.print(setPoint_RPM[0]);  Serial.print(" ");
  Serial.print(setPoint_RPM[1]);  Serial.print(" ");
  Serial.print(setPoint_RPM[2]);  Serial.print(" ");
  Serial.print(setPoint_RPM[3]);  Serial.print(" ");

  Serial.print(encoder_RPM[0]);  Serial.print(" ");
  Serial.print(encoder_RPM[1]);  Serial.print(" ");
  Serial.print(encoder_RPM[2]);  Serial.print(" ");
  Serial.print(encoder_RPM[3]);  Serial.print(" ");
  
  Serial.println();
}
