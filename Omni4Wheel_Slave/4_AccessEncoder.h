#ifndef __AE__
#define __AE__
void IRAM_ATTR encoderAll_RPM();
extern volatile int16_t encoder_cnt[4];
extern volatile int16_t encoder_prev_cnt[4];
extern volatile int16_t encoder_last_cnt[4];
extern volatile int16_t encoder_velocity[4];
extern volatile int16_t encoder_velocity_monitor[4];
#endif
