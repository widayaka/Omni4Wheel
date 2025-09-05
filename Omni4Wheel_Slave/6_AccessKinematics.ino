void drive(float speed_global_x, float speed_global_y, float speed_angular_w, float offset_heading, int number_of_wheels, float wheelRadius, float robotRadius, float *motorOut){
  // kecepatan (x,y,w) yang dimasukkan dalam satuan meter/sekon (m/s)
  float del_angle = 360.0f / number_of_wheels;
  
  for(int i = 0; i < number_of_wheels; i++){
    float angleDeg = del_angle * i + offset_heading;
    float angleRad = angleDeg * PI / 180.0f;

    motorOut[i] = (-sinf(angleRad) * speed_global_x) / wheelRadius;
    motorOut[i] += (cosf(angleRad) * speed_global_y) / wheelRadius;
    motorOut[i] += (robotRadius * speed_angular_w) / wheelRadius;
  }
}

void robotOodometry(float MI[2][4], float T[4], float angle, float output[2]) {
    float rad = angle * PI / 180.0f; 
    
    float R[2][2] = {
        {cosf(rad), -sinf(rad)},
        {sinf(rad),  cosf(rad)}
    };

    float M[2][1] = {0};
    
    for(int i = 0; i < 2; i++) {
        M[i][0] = 0;
        for(int j = 0; j < 4; j++) {
            M[i][0] += MI[i][j] * T[j];
        }
    }
    
    for(int i = 0; i < 2; i++) {
        output[i] = 0;
        for(int j = 0; j < 2; j++) {
            output[i] += R[i][j] * M[j][0];
            output_[i] = output[i];
        }
    }
}

void setRobotPosition(float x, float y, float w, float maxSpeedLin, float maxSpeedAng) {
  RobotSetPointX = x;
  RobotSetPointY = y;
  RobotSetPointTheta = w;
  maxSpeedRobotLin = maxSpeedLin;
  maxSpeedRobotAng = maxSpeedAng;
}

void setRobotSpeed(float x, float y, float w) {
  drive(x, y, w, OFFSET_HEADING, NUM_OF_MOTORS, R_WHEEL, R_ROBOT, setPoint_RPM);
}
