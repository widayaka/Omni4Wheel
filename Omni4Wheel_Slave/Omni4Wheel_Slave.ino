/*
 * Bismillahirrohmanirrohim
 * Project      : Omni 4 Wheel Slave Board Program
 * Author       : Parama Diptya Widayaka, S.ST., M.T.
 * Affiliation  : Universitas Negeri Surabaya
 *                Faculty of Engineering
 *                Department of Electrical Engineering
 *                Microprocessor Laboratory
 * Year         : 2025
 * Program Ver  : 1.0
 * 
 * PIN CONFIGURATION
 * 1. Push Button Up    -> GPIO Pin 5
 * 2. Push Button Down  -> GPIO Pin 3
 * 3. Push Button OK    -> GPIO Pin 15
 * 4. Push Button Run   -> GPIO Pin 12
 * 4. LED BUILTIN       -> GPIO Pin 2
 * 5. Encoder 1A        -> GPIO Pin 36
 * 6. Encoder 1B        -> GPIO Pin 39
 * 7. Encoder 2A        -> GPIO Pin 34
 * 8. Encoder 2B        -> GPIO Pin 35
 * 9. Encoder 3A        -> GPIO Pin 32
 * 10. Encoder 3B       -> GPIO Pin 33  
 * 11. Encoder 4A       -> GPIO Pin 25
 * 12. Encoder 4B       -> GPIO Pin 26
 */

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

#define NUM_OF_DATA 4
#define NUM_OF_MOTORS 4
#define NUM_OF_ENCODER 4
#define ENCODER_PPR 11
#define R_WHEEL 0.03
#define R_ROBOT 0.1

#define OFFSET_HEADING 45   // Posisi roda 1 berada pada sudut 45 derajat dari sumbu x sehingga arah hadap robot berada pada sumbu y (konfigurasi frame robot X)

#define PI 3.14159265359
#define CW 0
#define CCW 1

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
int16_t encoder_prev_cnt[NUM_OF_MOTORS];
int16_t encoder_last_cnt[NUM_OF_MOTORS];
int16_t encoder_velocity[NUM_OF_MOTORS];
int16_t DistanceTravelledByWheel[NUM_OF_MOTORS];

float speed_motor_encoder[NUM_OF_MOTORS];
float setPoint_velocity[NUM_OF_MOTORS];

float P[NUM_OF_MOTORS], P_w[NUM_OF_MOTORS];
float I[NUM_OF_MOTORS], I_w[NUM_OF_MOTORS];
float D[NUM_OF_MOTORS], D_w[NUM_OF_MOTORS];
float PIDForRPM[NUM_OF_MOTORS], PID_w[NUM_OF_MOTORS];

float KP[NUM_OF_MOTORS];
float KI[NUM_OF_MOTORS];
float KD[NUM_OF_MOTORS];

float motor_p;
float motor_i;
float motor_d;

float motor_pid_min;
float motor_pid_max;

float error[NUM_OF_MOTORS];
float lastError[NUM_OF_MOTORS];

float motor[NUM_OF_MOTORS];
float motor_setPoint[NUM_OF_MOTORS];
float pwm_motor[NUM_OF_MOTORS];

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

bool statusWaypointReached;

#define NUM_OF_WAYPOINTS 2
float robotWaypoint[NUM_OF_WAYPOINTS][4] = {
  {0.5, 0, 0, 100},
  {0, 0.5, 0, 50}
};

float RobotSetPointX = 0.0;
float RobotSetPointY = 0.0;
float RobotSetPointTheta = 0.0;
float RobotSetPointSpeed = 0.0;

float RobotActualPositionX = 0.0;
float RobotActualPositionY = 0.0;
float RobotActualPositionTheta = 0.0;

float DistanceRobotToTarget = 0.0;
float TotalDistanceTravelledByRobot = 0.0;

float VelocityRobotX = 0.0;
float VelocityRobotY = 0.0;
float VelocityRobotZ = 0.0;

float ErrorPositionX = 0.0;
float ErrorPositionY = 0.0;
float ErrorPositionTheta = 0.0;

float v_robot_l_x = 0.0;
float v_robot_l_y = 0.0;

unsigned long currentTime_odom, previousTime_odom;

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

hw_timer_t *timer20ms = NULL;
volatile bool flag_20ms = false;
void IRAM_ATTR onTimer() {flag_20ms = true;}

void IRAM_ATTR ontTimer20ms(){
  
}

TaskHandle_t Task_ReadencoderAll_RPM = NULL;
void encoderAll_RPM(void *parameter){
  for(;;){
    currentTime = millis();
    if (currentTime - previousTime >= interval){
      previousTime = currentTime;
      for(int i = 0; i < NUM_OF_MOTORS; i++){
        encoder_velocity[i] = ((encoder_cnt[i] - encoder_prev_cnt[i]) * 60) / ENCODER_PPR;
        encoder_prev_cnt[i] = encoder_cnt[i];
      }
    }
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

bool enableMotorControl;
TaskHandle_t Task_MotorRPMWithPID = NULL;
void MotorRPMWithPID(void *parameter){
  for(;;){
    if (!enableMotorControl) {
      vTaskDelay(20 / portTICK_PERIOD_MS); // tidur sebentar
      continue;
    }
    
    for (int i = 0; i < NUM_OF_MOTORS; i++){    
      setPoint_velocity[i] = velocity_wheel[i];
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
bool enablePositionControl;
TaskHandle_t Task_robotSetPosition = NULL;
void robotSetPosition(void *parameter){
  for(;;){
    if (!enablePositionControl) {
      vTaskDelay(20 / portTICK_PERIOD_MS);
      continue;
    }

    RobotSetPointX = robotWaypoint[currentWaypointIndex][0];
    RobotSetPointY = robotWaypoint[currentWaypointIndex][1];
    RobotSetPointTheta = robotWaypoint[currentWaypointIndex][2];
    float maxSpeed = robotWaypoint[currentWaypointIndex][3];
    
    errorPosXAxis = RobotSetPointX - RobotActualPositionX;
    errorPosYAxis = RobotSetPointY - RobotActualPositionY;
    errorPosTheta = RobotSetPointTheta - RobotActualPositionTheta;

    if (errorPosTheta > 180){errorPosTheta = errorPosTheta - 360;}
    else if (errorPosTheta < -180){errorPosTheta = errorPosTheta + 360;}

    TotalDistanceTravelledByRobot = sqrt((errorPosXAxis*errorPosXAxis) + (errorPosYAxis*errorPosYAxis));

    P_odom = KP_odom * TotalDistanceTravelledByRobot;

    VelocityRobotX = P_odom * errorPosXAxis / TotalDistanceTravelledByRobot;
    VelocityRobotY = P_odom * errorPosYAxis / TotalDistanceTravelledByRobot;
    VelocityRobotZ = 0;

    if (VelocityRobotX > maxSpeed) VelocityRobotX = maxSpeed;
    if (VelocityRobotY > maxSpeed) VelocityRobotY = maxSpeed;
    if (VelocityRobotZ > maxSpeed) VelocityRobotZ = maxSpeed;

    drive(VelocityRobotX, VelocityRobotY, VelocityRobotZ, maxSpeed, OFFSET_HEADING, NUM_OF_MOTORS, R_WHEEL, R_ROBOT, velocity_wheel);

    if (TotalDistanceTravelledByRobot < 0.01f){
      if (currentWaypointIndex < NUM_OF_WAYPOINTS - 1) {
        currentWaypointIndex++;   // pindah ke waypoint berikutnya
      } 
      
      else {
        // sudah di waypoint terakhir → stop
        VelocityRobotX = 0;
        VelocityRobotY = 0;
        VelocityRobotZ = 0;
        drive(0, 0, 0, 0, OFFSET_HEADING, NUM_OF_MOTORS, R_WHEEL, R_ROBOT, velocity_wheel);
        SetPointRPM(0, 0, 0, 0);
      }
    }
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

  timer20ms = timerBegin(0, 80, true);
  timerAttachInterrupt(timer20ms, &onTimer, true);
  timerAlarmWrite(timer20ms, 20000, true);
  timerAlarmEnable(timer20ms);

  xTaskCreatePinnedToCore(
    encoderAll_RPM,               // Task function
    "encoderAll_RPM",             // Task name
    4096,                        // Stack size (bytes)
    NULL,                         // Parameters
    1,                            // Priority
    &Task_ReadencoderAll_RPM,     // Task handle
    0                             // Core
  );

  xTaskCreatePinnedToCore(
    MotorRPMWithPID,              // Task function
    "MotorRPMWithPID",            // Task name
    4096,                        // Stack size (bytes)
    NULL,                         // Parameters
    1,                            // Priority
    &Task_MotorRPMWithPID,        // Task handle
    0                             // Core 1
  );

  xTaskCreatePinnedToCore(
    odometryTimerLoop,            // Task function
    "odometryTimerLoop",          // Task name
    4096,                        // Stack size (bytes)
    NULL,                         // Parameters
    1,                            // Priority
    &Task_odometryTimerLoop,      // Task handle
    0                             // Core 1
  );

  xTaskCreatePinnedToCore(
    robotSetPosition,            // Task function
    "robotSetPosition",          // Task name
    4096,                        // Stack size (bytes)
    NULL,                         // Parameters
    1,                            // Priority
    &Task_robotSetPosition,      // Task handle
    0                             // Core 1
  );

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
  SetPIDMotor(0.01, 0.1, 0.001);
  SetPointRPM(250, 250, 250, 250);
  
  enablePositionControl = true;
  SetPIDGainOdomRobot(100,0,0);
  
  SetPIDGainOdomX(0,0,0);
  SetPIDGainOdomY(0,0,0);
  SetPIDGainOdomTheta(0,0,0);
  SetPIDGainOdomRobot(200,0,0);
  
  for(int i = 0; i < NUM_OF_MOTORS; i++){encoder_cnt[i] = 0;}
  previousTime_odom = millis();
  RobotBootScreen();
}

void loop() {
//  while (menu == 0) {RobotHomeScreen();}
//  while (menu == 1) {RobotMenuEncoder();}
//  while (menu == 2) {RobotMenuMotor();}

//  while (menu == 6) {RobotOdometry();}
//  while (menu == 7) {RobotHoldPosition();}
//  while (menu == 8) {RobotJoystickControl();}
//  while (menu == 9) {RobotOdometry();}

//Serial.print(currentWaypointIndex); Serial.print(" "); 
//Serial.print(TotalDistanceTravelledByRobot); Serial.print(" "); 
//Serial.print(errorPosXAxis); Serial.print(" ");
//Serial.print(errorPosYAxis); Serial.print(" ");
//Serial.print(encoder_velocity[0]); Serial.print(" ");
//Serial.print(encoder_velocity[1]); Serial.print(" ");
//Serial.print(encoder_velocity[2]); Serial.print(" ");
//Serial.print(encoder_velocity[3]); Serial.print(" ");
//Serial.print(setPoint_velocity[0]); Serial.print(" ");
//Serial.print(setPoint_velocity[1]); Serial.print(" ");
//Serial.print(setPoint_velocity[2]); Serial.print(" ");
//Serial.print(setPoint_velocity[3]); Serial.print(" ");
//Serial.println();
//delay(100);
}
