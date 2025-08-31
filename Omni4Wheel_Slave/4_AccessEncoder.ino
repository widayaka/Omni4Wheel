volatile int16_t encoder_cnt[4];
volatile int16_t encoder_prev_cnt[4];
volatile int16_t encoder_last_cnt[4];
volatile int16_t encoder_velocity[4];
volatile int16_t encoder_velocity_monitor[4];

void encoder1_ISR(){
  VAL_ENC_1A = digitalRead(PIN_ENC_1A);
  VAL_ENC_1B = digitalRead(PIN_ENC_1B);
  if ((VAL_ENC_1A == HIGH) != (VAL_ENC_1B == LOW)){encoder_cnt[0]++;}
  else{encoder_cnt[0]--;}
}

void encoder2_ISR(){
  VAL_ENC_2A = digitalRead(PIN_ENC_2A);
  VAL_ENC_2B = digitalRead(PIN_ENC_2B);
  if ((VAL_ENC_2A == HIGH) != (VAL_ENC_2B == LOW)){encoder_cnt[1]++;}
  else{encoder_cnt[1]--;}
}

void encoder3_ISR(){
  VAL_ENC_3A = digitalRead(PIN_ENC_3A);
  VAL_ENC_3B = digitalRead(PIN_ENC_3B);
  if ((VAL_ENC_3A == HIGH) != (VAL_ENC_3B == LOW)){encoder_cnt[2]++;}
  else{encoder_cnt[2]--;}
}

void encoder4_ISR(){
  VAL_ENC_4A = digitalRead(PIN_ENC_4A);
  VAL_ENC_4B = digitalRead(PIN_ENC_4B);
  if ((VAL_ENC_4A == HIGH) != (VAL_ENC_4B == LOW)){encoder_cnt[3]++;}
  else{encoder_cnt[3]--;}
}

void encoder1_RPM(){
  currentTime = millis();
  if (currentTime - previousTime >= interval){
    previousTime = currentTime;
    encoder_velocity[0] = ((encoder_cnt[0] - encoder_prev_cnt[0]) * 60) / ENCODER_PPR;
    encoder_prev_cnt[0] = encoder_cnt[0];
  }
}

void encoder2_RPM(){
  currentTime = millis();
  if (currentTime - previousTime >= interval){
    previousTime = currentTime;
    encoder_velocity[1] = ((encoder_cnt[1] - encoder_prev_cnt[1]) * 60) / ENCODER_PPR;
    encoder_prev_cnt[1] = encoder_cnt[1];
  }
}

void encoder3_RPM(){
  currentTime = millis();
  if (currentTime - previousTime >= interval){
    previousTime = currentTime;
    encoder_velocity[2] = ((encoder_cnt[2] - encoder_prev_cnt[2]) * 60) / ENCODER_PPR;
    encoder_prev_cnt[2] = encoder_cnt[2];
  }
}

void encoder4_RPM(){
  currentTime = millis();
  if (currentTime - previousTime >= interval){
    previousTime = currentTime;
    encoder_velocity[3] = ((encoder_cnt[3] - encoder_prev_cnt[3]) * 60) / ENCODER_PPR;
    encoder_prev_cnt[3] = encoder_cnt[3];
  }
}

void IRAM_ATTR encoderAll_RPM(){
  currentTime = millis();
  if (currentTime - previousTime >= interval){
    previousTime = currentTime;
    for(int i = 0; i < NUM_OF_MOTORS; i++){
      encoder_velocity[i] = ((encoder_cnt[i] - encoder_prev_cnt[i]) * 60) / ENCODER_PPR;
      encoder_prev_cnt[i] = encoder_cnt[i];
    }
  }
}
