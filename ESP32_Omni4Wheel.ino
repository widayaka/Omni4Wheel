#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Ps3Controller.h>

#define NUM_OF_MOTORS 4
#define NUM_OF_ENCODER 4
#define ENCODER_PPR 11

#define JOYSTICK_MACADDRESS "4a:e5:4a:ec:a1:00"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

int PB_RUN = 12;
int PB_UP = 5;
int PB_DOWN = 3;
int PB_OK = 15;

int16_t encoder_cnt[NUM_OF_MOTORS];
int16_t encoder_last_cnt[NUM_OF_MOTORS];
int16_t encoder_velocity[NUM_OF_MOTORS];
int16_t encoder_velocity_monitor[NUM_OF_MOTORS];

float motor[NUM_OF_MOTORS];
float motor_setPoint[NUM_OF_MOTORS];
float pwm_motor[NUM_OF_MOTORS];

float motor_p;
float motor_i;
float motor_d;

float motor_pid_min;
float motor_pid_max;

int rpm1;

int MOTOR_1A = 23;
int MOTOR_1B = 19;
int ENC_MOTOR_1A = 36;
int ENC_MOTOR_1B = 39;
int VAL_ENC_MOTOR_1A = 0;
int VAL_ENC_MOTOR_1B = 0;

int MOTOR_2A = 18;
int MOTOR_2B = 4;
int ENC_MOTOR_2A = 34;
int ENC_MOTOR_2B = 35;
int VAL_ENC_MOTOR_2A = 0;
int VAL_ENC_MOTOR_2B = 0;

int MOTOR_3A = 2;
int MOTOR_3B = 13;
int ENC_MOTOR_3A = 32;
int ENC_MOTOR_3B = 33;
int VAL_ENC_MOTOR_3A = 0;
int VAL_ENC_MOTOR_3B = 0;

int MOTOR_4A = 14;
int MOTOR_4B = 27;
int ENC_MOTOR_4A = 25;
int ENC_MOTOR_4B = 26;
int VAL_ENC_MOTOR_4A = 0;
int VAL_ENC_MOTOR_4B = 0;

unsigned long currentTime, previousTime = 0;
int interval = 20;

int frequency = 1000;
int resolution  = 8;

int pwmChannel1 = 0, 
    pwmChannel2 = 1, 
    pwmChannel3 = 2, 
    pwmChannel4 = 3;

int pilih = 0;
int x_error_data;

void setup() {
  Serial.begin(9600);
  pinMode(PB_RUN, INPUT_PULLUP);
  pinMode(PB_UP, INPUT_PULLUP);
  pinMode(PB_DOWN, INPUT_PULLUP);
  pinMode(PB_OK, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  ledcSetup(pwmChannel1, frequency, resolution);
  ledcAttachPin(MOTOR_1A, pwmChannel1);
  pinMode(MOTOR_1B, OUTPUT);
//  pinMode(ENC_MOTOR_1A, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_MOTOR_1A), Access_Encoder_Motor_1, CHANGE);
  pinMode(ENC_MOTOR_1B, INPUT);
  encoder_last_cnt[0] = digitalRead(ENC_MOTOR_1A);
  
  ledcSetup(pwmChannel2, frequency, resolution);
  ledcAttachPin(MOTOR_2A, pwmChannel2);
  pinMode(MOTOR_2B, OUTPUT);
//  pinMode(ENC_MOTOR_2A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_MOTOR_2A), Access_Encoder_Motor_2, CHANGE);
  pinMode(ENC_MOTOR_2B, INPUT_PULLUP);
  encoder_last_cnt[1] = digitalRead(ENC_MOTOR_2A);

  ledcSetup(pwmChannel3, frequency, resolution);
  ledcAttachPin(MOTOR_3A, pwmChannel3);
  pinMode(MOTOR_3B, OUTPUT);
//  pinMode(ENC_MOTOR_3A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_MOTOR_3A), Access_Encoder_Motor_3, CHANGE);
  pinMode(ENC_MOTOR_3B, INPUT_PULLUP);
  encoder_last_cnt[2] = digitalRead(ENC_MOTOR_3A);

  ledcSetup(pwmChannel4, frequency, resolution);
  ledcAttachPin(MOTOR_4A, pwmChannel4);
  pinMode(MOTOR_4B, OUTPUT);
//  pinMode(ENC_MOTOR_4A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_MOTOR_4A), Access_Encoder_Motor_4, CHANGE);
  pinMode(ENC_MOTOR_4B, INPUT_PULLUP);
  encoder_last_cnt[3] = digitalRead(ENC_MOTOR_4A);

  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  
  robot_boot_screen();
}

void loop() {
//   drive_motor_4(255);
//   Access_RPM_Motor_4();
   Control_Motor_Speed(10, 0.75, 0, 0);
//   Serial.println(encoder_cnt[3]);
}
