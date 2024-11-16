void Access_Encoder_Motor_1(){
  VAL_ENC_MOTOR_1A = digitalRead(ENC_MOTOR_1A);
  VAL_ENC_MOTOR_1B = digitalRead(ENC_MOTOR_1B);
  if(VAL_ENC_MOTOR_1A != encoder_last_cnt[0]){
    if(VAL_ENC_MOTOR_1B != VAL_ENC_MOTOR_1A) {encoder_cnt[0] = encoder_cnt[0] - 1;}
    else{encoder_cnt[0] = encoder_cnt[0] + 1;}
    }
  encoder_last_cnt[0] = VAL_ENC_MOTOR_1A;
}

void Access_Encoder_Motor_2(){
  VAL_ENC_MOTOR_2A = digitalRead(ENC_MOTOR_2A);
  VAL_ENC_MOTOR_2B = digitalRead(ENC_MOTOR_2B);
  if(VAL_ENC_MOTOR_2A != encoder_last_cnt[1]){
    if(VAL_ENC_MOTOR_2B != VAL_ENC_MOTOR_2A) {encoder_cnt[1] = encoder_cnt[1] - 1;}
    else{encoder_cnt[1] = encoder_cnt[1] + 1;}
  }
  encoder_last_cnt[1] = VAL_ENC_MOTOR_2A;
}

void Access_Encoder_Motor_3(){
  VAL_ENC_MOTOR_3A = digitalRead(ENC_MOTOR_3A);
  VAL_ENC_MOTOR_3B = digitalRead(ENC_MOTOR_3B);
  if(VAL_ENC_MOTOR_3A != encoder_last_cnt[2]){
    if(VAL_ENC_MOTOR_3B != VAL_ENC_MOTOR_3A) {encoder_cnt[2] = encoder_cnt[2] - 1;}
    else{encoder_cnt[2] = encoder_cnt[2] + 1;}
  }
  encoder_last_cnt[2] = VAL_ENC_MOTOR_3A;
}

void Access_Encoder_Motor_4(){
  VAL_ENC_MOTOR_4A = digitalRead(ENC_MOTOR_4A);
  VAL_ENC_MOTOR_4B = digitalRead(ENC_MOTOR_4B);
  if(VAL_ENC_MOTOR_4A != encoder_last_cnt[3]){
    if(VAL_ENC_MOTOR_4B != VAL_ENC_MOTOR_4A) {encoder_cnt[3] = encoder_cnt[3] - 1;}
    else{encoder_cnt[3] = encoder_cnt[3] + 1;}
  }
  encoder_last_cnt[3] = VAL_ENC_MOTOR_4A;
}

void Access_RPM_Motor_1(){
  currentTime = millis();
  if (currentTime - previousTime >= interval){
    previousTime = currentTime;
    encoder_velocity[0] = (encoder_cnt[0] * 60) / 11;
//    Serial.print("RPM 1: "); Serial.println(encoder_velocity[0]);
    encoder_cnt[0] = 0;
  }
}

void Access_RPM_Motor_2(){
  currentTime = millis();
  if (currentTime - previousTime >= interval){
    previousTime = currentTime;
    encoder_velocity[1] = (encoder_cnt[1] * 60) / 11;
//     Serial.print("RPM 2: "); Serial.println(encoder_velocity[1]);
    encoder_cnt[1] = 0;
  }
}

void Access_RPM_Motor_3(){
  currentTime = millis();
  if (currentTime - previousTime >= interval){
    previousTime = currentTime;
    encoder_velocity[2] = (encoder_cnt[2] * 60) / 11;
//     Serial.print("RPM 3: "); Serial.println(encoder_velocity[2]);
    encoder_cnt[2] = 0;
  }
}

void Access_RPM_Motor_4(){
  currentTime = millis();
  if (currentTime - previousTime >= interval){
    previousTime = currentTime;
    encoder_velocity[3] = (encoder_cnt[3] * 60) / 11;
//     Serial.print("RPM 4: "); Serial.println(encoder_velocity[3]);
    encoder_cnt[3] = 0;
  }
}

void Control_Motor_Speed(int setPoint_velocity, float kp, float ki, float kd){
  Access_RPM_Motor_1();
  float dt = 1;
  float lastError;
  float error = setPoint_velocity-encoder_velocity[0];
  float P = (float)(error*kp); 
  float I = I+(float)error*ki*dt;
  float D = (float)(lastError-error)*kd/dt;
  float PIDOut = P+I+D;

  if (I > 255) I = 255;
  if (I < -255) I = -255;
  
  // Serial.print(setPoint); Serial.print("\t");
  Serial.print(encoder_velocity[0]); Serial.println("\t");  
  // Serial.print(error); Serial.print("\t"); 
  // Serial.println(PIDOut);
  
  drive_motor_1(PIDOut);
  lastError = error;
}
