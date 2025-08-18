void motor1(int PWM){
  int dir;
  dir = PWM > 0 ? CW : CCW;
  if (PWM > 255) PWM = 255;
  else if (PWM < 0) {PWM = -PWM; PWM = 255 - PWM;}
  else if (PWM == 0) {PWM = PWM; dir = 0;}

  ledcWrite(pwmChannel1, PWM); 
  digitalWrite(PIN_MOT_1B, dir);
}

void motor2(int PWM){
  int dir;
  dir = PWM > 0 ? CW : CCW;
  if (PWM > 255) PWM = 255;
  else if (PWM < 0) {PWM = -PWM; PWM = 255 - PWM;}
  else if (PWM == 0) {PWM = PWM; dir = 0;}

  ledcWrite(pwmChannel2, PWM); 
  digitalWrite(PIN_MOT_2B, dir);
}

void motor3(int PWM){
  int dir;
  dir = PWM > 0 ? CW : CCW;
  if (PWM > 255) PWM = 255;
  else if (PWM < 0) {PWM = -PWM; PWM = 255 - PWM;}
  else if (PWM == 0) {PWM = PWM; dir = 0;}

  ledcWrite(pwmChannel3, PWM); 
  digitalWrite(PIN_MOT_3B, dir);
}

void motor4(int PWM){
  int dir;
  dir = PWM > 0 ? CW : CCW;
  if (PWM > 255) PWM = 255;
  else if (PWM < 0) {PWM = -PWM; PWM = 255 - PWM;}
  else if (PWM == 0) {PWM = PWM; dir = 0;}

  ledcWrite(pwmChannel4, PWM); 
  digitalWrite(PIN_MOT_4B, dir);
}

void DriveMotor(int speedVal1, int speedVal2, int speedVal3, int speedVal4){
  motor1(speedVal1);
  motor2(speedVal2);
  motor3(speedVal3);
  motor4(speedVal4);
}
