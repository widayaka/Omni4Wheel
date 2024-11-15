void drive_motor_1(int speedValue){ 
  if (speedValue > 255) speedValue=255;
  if (speedValue > 0) {
    ledcWrite(pwmChannel1, speedValue);
    digitalWrite(MOTOR_1B, LOW);
  }
  if (speedValue < -255) speedValue=-255;
  if (speedValue < 0){
    ledcWrite(pwmChannel1, speedValue+255);
    digitalWrite(MOTOR_1B, HIGH);
  }
}

void drive_motor_2(int speedValue){
  if (speedValue > 255) speedValue=255;
  if (speedValue > 0) {
    ledcWrite(pwmChannel2, speedValue);
    digitalWrite(MOTOR_2B, LOW);
  }
  if (speedValue < -255) speedValue=-255;
  if (speedValue < 0){
    ledcWrite(pwmChannel2, speedValue+255);
    digitalWrite(MOTOR_2B, HIGH);
  }
}

void drive_motor_3(int speedValue){
  if (speedValue > 255) speedValue=255;
  if (speedValue > 0) {
    ledcWrite(pwmChannel3, speedValue);
    digitalWrite(MOTOR_3B, LOW);
  }
  if (speedValue < -255) speedValue=-255;
  if (speedValue < 0){
    ledcWrite(pwmChannel3, speedValue+255);
    digitalWrite(MOTOR_3B, HIGH);
  }
}

void drive_motor_4(int speedValue){
  if (speedValue > 255) speedValue=255;
  if (speedValue > 0) {
    ledcWrite(pwmChannel4, speedValue);
    digitalWrite(MOTOR_4B, LOW);
  }
  if (speedValue < -255) speedValue=-255;
  if (speedValue < 0){
    ledcWrite(pwmChannel4, speedValue+255);
    digitalWrite(MOTOR_4B, HIGH);
  }
}

void Set_Motor_PWM(int motor1, int motor2, int motor3, int motor4){
  drive_motor_1(motor1); drive_motor_2(motor2); drive_motor_3(motor3); drive_motor_4(motor4);
}

void move_forward(int speed){
  drive_motor_1(speed);
  drive_motor_2(-speed);
  drive_motor_3(-speed);
  drive_motor_4(speed);
}

void move_backward(int speed){
  drive_motor_1(-speed);
  drive_motor_2(speed);
  drive_motor_3(speed);
  drive_motor_4(-speed);
}

void move_right(int speed){
  drive_motor_1(-speed);
  drive_motor_2(-speed);
  drive_motor_3(speed);
  drive_motor_4(speed);
}

void move_left(int speed){
  drive_motor_1(speed);
  drive_motor_2(speed);
  drive_motor_3(speed);
  drive_motor_4(-speed);
}
