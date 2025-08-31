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
#include "9_SerialPrintWrapper.h"
#include "8_GlobalControl.h"
#include "4_AccessEncoder.h"
#include "6_AccessKinematics.h"
#include "5_AccessMotor.h"

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

//#define OFFSET_HEADING 0    // Posisi roda 1 berada pada sudut 0 derajat dari sumbu x sehingga arah hadap robot berada pada sumbu x (konfigurasi frame robot +)
#define OFFSET_HEADING 45   // Posisi roda 1 berada pada sudut 45 derajat dari sumbu x sehingga arah hadap robot berada pada sumbu y (konfigurasi frame robot X)
//#define OFFSET_HEADING -45  // Posisi roda 1 berada pada sudut -45 derajat dari sumbu x sehingga arah hadap robot berada pada sumbu x (konfigurasi frame robot X)

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

int PB_RUN = 12;
int PB_UP = 5;
int PB_DOWN = 3;
int PB_OK = 15;

// PID Control Variables for Encoders

int16_t DistanceTravelledByWheel[NUM_OF_MOTORS];

float speed_motor_encoder[NUM_OF_MOTORS];

//int16_t encoder_cnt[NUM_OF_MOTORS];
//int16_t encoder_prev_cnt[NUM_OF_MOTORS];
//int16_t encoder_last_cnt[NUM_OF_MOTORS];
//int16_t encoder_velocity[NUM_OF_MOTORS];
//int16_t encoder_velocity_monitor[NUM_OF_MOTORS];

float P[NUM_OF_MOTORS], P_w[NUM_OF_MOTORS],
      I[NUM_OF_MOTORS], I_w[NUM_OF_MOTORS],
      D[NUM_OF_MOTORS], D_w[NUM_OF_MOTORS],
      PIDForRPM[NUM_OF_MOTORS], PID_w[NUM_OF_MOTORS];

float KP[NUM_OF_MOTORS],
      KI[NUM_OF_MOTORS],
      KD[NUM_OF_MOTORS];

float motor_pid_min,
      motor_pid_max;

float motor[NUM_OF_MOTORS],
      motor_setPoint[NUM_OF_MOTORS],
      pwm_motor[NUM_OF_MOTORS];

// PID Control Variables for Heading Lock
float setPointYaw,
      errorYaw,
      lastErrorYaw,
      P_yaw,
      I_yaw,
      D_yaw,
      PID_yaw;

float yaw_p,
      yaw_i,
      yaw_d;

float yaw_pid_min,
      yaw_pid_max;

// PID Control Variables for Odometry
float P_xAxis, I_xAxis, D_xAxis, 
      KP_xAxis, KI_xAxis, KD_xAxis, 
      errorPosXAxis, lastErrorPosXAxis, 
      PID_xAxis;
      
float P_yAxis, I_yAxis, D_yAxis, 
      KP_yAxis, KI_yAxis, KD_yAxis, 
      errorPosYAxis, lastErrorPosYAxis,
      PID_yAxis;
      
float P_theta, I_theta, D_theta,
      KP_theta, KI_theta, KD_theta, 
      errorPosTheta, lastErrorPosTheta,
      PID_theta;

float RobotActualPositionX = 0.0;
float RobotActualPositionY = 0.0;
float RobotActualPositionTheta = 0.0;
// float DistanceTravelledByRobot = 0.0;
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

float velocity_wheel[4],
      velocity_robot[2],
      output_[2],
      theta_dot[4];
      
float yaw_robot,
      yaw_polar;

int deltaTime_odom,
    delta_encoder[4];

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

int frequency=5000,
    resolution=8,
    pwmChannel1 = 0, 
    pwmChannel2 = 1, 
    pwmChannel3 = 2, 
    pwmChannel4 = 3;

unsigned long currentTime, previousTime = 0;
int interval = 20;

int menu=0, 
    pilih=0, 
    pilih_menu_encoder=0, 
    pilih_menu_motor=0;
    
// Serial Communication Properties
const char startDataIdentifier = '*',
           dataSeparator = ',',
           stopDataIdentifier = '#';

int16_t sendDataEncoder1, 
        sendDataEncoder2, 
        sendDataEncoder3, 
        sendDataEncoder4;
    
int16_t sendDataRPM1, 
        sendDataRPM2, 
        sendDataRPM3, 
        sendDataRPM4;

char dataReceived;

const byte arraySize = 21;

String dataPackage,
       bufferDataIn,
       bufferDataParsing[arraySize];

bool dataIsComplete;

byte indexOfData;

int counter;
float SerialData[10];
float data1, data2, data3, data4;
int UARTStatusDisplay;

hw_timer_t *timer20ms = NULL;
volatile bool flag_20ms = false;
void IRAM_ATTR onTimer() {flag_20ms = true;}

void IRAM_ATTR onTimerInterrupt20ms() { //control loop
  // get Motor RPM
  encoderAll_RPM();
  // odometry
  odometryTimerLoop();
  // kontrol motor
  globalMotorControl();
  // kontrol posisi
//  globalPositionControl();
}

void setup() {
  Serial.begin(115200);
  SlaveSerial2.begin(SLAVE_SERIAL_BAUDRATE, SERIAL_8N1, RX2, TX2);
  
  pinMode(PB_RUN, INPUT_PULLUP);
  pinMode(PB_UP, INPUT_PULLUP);
  pinMode(PB_DOWN, INPUT_PULLUP);
  pinMode(PB_OK, INPUT_PULLUP);

  timer20ms = timerBegin(0, 80, true);
  timerAttachInterrupt(timer20ms, &onTimerInterrupt20ms, true);
  timerAlarmWrite(timer20ms, 20000, true);
  timerAlarmEnable(timer20ms);

  ledcSetup(pwmChannel1, frequency, resolution);
  ledcAttachPin(PIN_MOT_1A, pwmChannel1); pinMode(PIN_MOT_1B, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_1A), encoder1_ISR, CHANGE); pinMode(PIN_ENC_1B, INPUT);
  
  ledcSetup(pwmChannel2, frequency, resolution);
  ledcAttachPin(PIN_MOT_2A, pwmChannel2); pinMode(PIN_MOT_2B, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_2A), encoder2_ISR, CHANGE); pinMode(PIN_ENC_2B, INPUT_PULLUP);

  ledcSetup(pwmChannel3, frequency, resolution);
  ledcAttachPin(PIN_MOT_3A, pwmChannel3); pinMode(PIN_MOT_3B, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_3A), encoder3_ISR, CHANGE); pinMode(PIN_ENC_3B, INPUT_PULLUP);

  ledcSetup(pwmChannel4, frequency, resolution);
  ledcAttachPin(PIN_MOT_4A, pwmChannel4); pinMode(PIN_MOT_4B, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_4A), encoder4_ISR, CHANGE); pinMode(PIN_ENC_4B, INPUT_PULLUP);

  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }

  SetPIDMinMax(-255, 255);
  SetPIDGainYaw(10, 0, 5);
  
  setPIDMotor(0.01,0.1,0);
  
  SetPIDGainOdomX(100,0,0);
  SetPIDGainOdomY(100,0,0);
  SetPIDGainOdomTheta(5,0,0);
  
  for(int i = 0; i < NUM_OF_MOTORS; i++){encoder_cnt[i] = 0;}
  previousTime_odom = millis();
  RobotBootScreen();
  setMotorRPM(10, 10, 10, 10);
//   enableMotorControl = false;
  enablePositionControl = false;
}

void loop() {
  checkSetPointMotor();
  SerialPrint("%d %d %d %d", encoder_velocity[0], encoder_velocity[1], encoder_velocity[2], encoder_velocity[3]);
  delay(10);
//  Serial.println("test");
  
//  while (menu == 0) {RobotHomeScreen();}
//  while (menu == 1) {RobotMenuEncoder();}
//  while (menu == 2) {RobotMenuMotor();}
//
//  while (menu == 6) {RobotOdometry();}
//  while (menu == 7) {RobotHoldPosition();}
//  while (menu == 8) {RobotJoystickControl();}
//  while (menu == 9) {RobotOdometry();}

//  if (flag_20ms) {flag_20ms = false; odometryTimerLoop();}
//  setRobotPosition(10,0,0,50);
//RobotOdometry();

//  Serial.print(theta_dot[0]); Serial.print("\t");
//  Serial.print(theta_dot[1]); Serial.print("\t");
//  Serial.print(theta_dot[2]); Serial.print("\t");
//  Serial.print(theta_dot[3]); Serial.print("\t");
//
//  Serial.print(velocity_wheel[0],2); Serial.print("\t");
//  Serial.print(velocity_wheel[1],2); Serial.print("\t");
//  Serial.print(velocity_wheel[2],2); Serial.print("\t");
//  Serial.print(velocity_wheel[3],2); Serial.print("\t");
//
//  Serial.print(velocity_robot[0],2); Serial.print("\t");
//  Serial.print(velocity_robot[1],2); Serial.print("\t");
//
//  Serial.print(v_robot_l_x); Serial.print("\t");
//  Serial.print(RobotActualPositionTheta); Serial.print("\t");
//
//  Serial.print(RobotActualPositionX); Serial.print("\t");
//  Serial.print(RobotActualPositionY); Serial.print("\t");
//  Serial.println();

//  delay(100);

//InverseKinematicsNoPID(10,0,0,50,OFFSET_HEADING,NUM_OF_MOTORS,R_WHEEL,R_ROBOT); //robot bergerak ke kanan (sumbu x+)
//delay(500);
//
//InverseKinematicsNoPID(0,10,0,50,OFFSET_HEADING,NUM_OF_MOTORS,R_WHEEL,R_ROBOT); //robot bergerak ke depan (sumbu y+) 
//delay(500);
//
//InverseKinematicsNoPID(-10,0,0,50,OFFSET_HEADING,NUM_OF_MOTORS,R_WHEEL,R_ROBOT); //robot bergerak ke kiri (sumbu x-)
//delay(500);
//
//InverseKinematicsNoPID(0,-10,0,50,OFFSET_HEADING,NUM_OF_MOTORS,R_WHEEL,R_ROBOT); //robot bergerak ke belakang (sumbu y-)
//delay(500);
}
