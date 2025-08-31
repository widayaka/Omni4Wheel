//float m_kinematic[4][3] = {
//    {-1.178511f, 1.178511f, 1.666667f},
//    {-1.178511f, -1.178511f, 1.666667f},
//    {1.178511f, -1.178511f, 1.666667f},
//    {1.178511f, 1.178511f, 1.666667f}
//};

//void transformKinematicsByMatrix(float x, float y, float w, const float matrixKinematics[4][3], float output[4]){
//  for (int i = 0; i < 4; i++){
//    output[i] = matrixKinematics[i][0] * x +
//                matrixKinematics[i][1] * y +
//                matrixKinematics[i][2] * w;
//  }
//}

// Matrix Odometry : N = 4, Heading Offset = 45, Robot Radius = 1.0, Wheel Radius = 0.6
//float m_odometry[2][4] = {
//    {-0.212132f, -0.212132f, 0.212132f, 0.212132f},
//    {0.212132f, -0.212132f, -0.212132f, 0.212132f}
//};

// Matrix Odometry : N = 4, Heading Offset = 45, Robot Radius = 1.0, Wheel Radius = 0.06
//float m_odometry[2][4] = {
//    {-0.021213f, -0.021213f, 0.021213f, 0.021213f},
//    {0.021213f, -0.021213f, -0.021213f, 0.021213f}
//};

// Matrix Odometry : N = 4, Heading Offset = 45, Robot Radius = 1.0, Wheel Radius = 0.03
float m_odometry[2][4] = {
    {-0.010607f, -0.010607f, 0.010607f, 0.010607f},
    {0.010607f, -0.010607f, -0.010607f, 0.010607f}
};

void drive(float x, float y, float w, float headingOffset, int n, float wheelRadius, float robotRadius, float* motorOutput) {
  float del_angle = 360.0f / n;

  for (int i = 0; i < n; i++) {
    float angleDeg = del_angle * i + headingOffset;
    float angleRad = angleDeg * M_PI / 180.0f;

    motorOutput[i] = (-x * sinf(angleRad)) / wheelRadius;
    motorOutput[i] += (y * cosf(angleRad)) / wheelRadius;
    motorOutput[i] += (w * robotRadius) / wheelRadius;
  }
}

void InverseKinematicsNoPID(float speed_global_x, float speed_global_y, float speed_angular_w, float maxSpeed, float offset_heading, int number_of_wheels, float wheelRadius, float robotRadius){
  float del_angle = 360.0f / number_of_wheels;
  float motor[number_of_wheels];
  
  for(int i = 0; i < number_of_wheels; i++){
    float angleDeg = del_angle * i + offset_heading;
    float angleRad = angleDeg * PI / 180.0f;
    
    motor[i] = (-sinf(angleRad) * speed_global_x) / wheelRadius;
    motor[i] += (cosf(angleRad) * speed_global_y) / wheelRadius;
    motor[i] += (robotRadius * speed_angular_w) / wheelRadius;

    if (motor[i] > maxSpeed)  motor[i] = maxSpeed;
    if (motor[i] < -maxSpeed) motor[i] = -maxSpeed;
    
    if (motor[i] > motor_pid_max) motor[i] = motor_pid_max;
    if (motor[i] < motor_pid_min) motor[i] = motor_pid_min;
  }
  
  DriveMotor(motor[0], 
             motor[1], 
             motor[2], 
             motor[3]);
}

//void InverseKinematicsWithPID(float speed_global_x, float speed_global_y, float speed_angular_w, float maxSpeed, float offset_heading, int number_of_wheels, float wheelRadius, float robotRadius){
//  float del_angle = 360.0f / number_of_wheels;
//  float motor[number_of_wheels];
//  
//  encoderAll_RPM();
//  
//  for(int i = 0; i < number_of_wheels; i++){
//    float angleDeg = (del_angle * i) + offset_heading;
//    float angleRad = angleDeg * PI / 180.0f;
//    
//    encoderAll_RPM();
//    motor[i] = (-speed_global_x * sinf(angleRad)) / wheelRadius;
//    motor[i] += (speed_global_y * cosf(angleRad)) / wheelRadius;
//    motor[i] += (speed_angular_w * robotRadius) / wheelRadius;
//      
//    if (motor[i] > maxSpeed)  motor[i] = maxSpeed;
//    if (motor[i] < -maxSpeed) motor[i] = -maxSpeed;
//
//    error[i] = (motor[i] - encoder_velocity[i]);
//    P[i] = error[i] * motor_p;
//    I[i] += error[i] * motor_i;
//    D[i] = (error[i] - lastError[i]) * motor_d;
//    PIDForRPM[i] = P[i] + I[i] + D[i];
//
//    if (PIDForRPM[i] > motor_pid_max) PIDForRPM[i] = motor_pid_max;
//    if (PIDForRPM[i] < motor_pid_min) PIDForRPM[i] = motor_pid_min;
//
//    if (I[i] > motor_pid_max) I[i] = motor_pid_max;
//    if (I[i] < motor_pid_min) I[i] = motor_pid_min;
//
//    if (error[i] == 0) I[i] = 0;
//    
//    lastError[i] = error[i];
//  }
//  
//  DriveMotor(PIDForRPM[0], PIDForRPM[1], PIDForRPM[2], PIDForRPM[3]);
//
//  display.clearDisplay(); display.setTextSize(1); display.setTextColor(SSD1306_WHITE);
//
//  display.setCursor(0,0);   display.print("SP RPM: ");  
//  display.setCursor(45,0);  display.print(motor[0]);
//  
//  display.setCursor(0,10);  display.print("RPM1: ");
//  display.setCursor(30,10);  display.print(encoder_velocity[0]);
//
//  display.setCursor(60,10);  display.print("RPM2: ");
//  display.setCursor(90,10);  display.print(encoder_velocity[1]);
//
//  display.setCursor(0,20);  display.print("RPM3: ");
//  display.setCursor(30,20);  display.print(encoder_velocity[2]);
//
//  display.setCursor(60,20);  display.print("RPM4: ");
//  display.setCursor(90,20);  display.print(encoder_velocity[3]);
//  
//  display.display();
//}

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

void odometryTimerLoop(){
  currentTime_odom = millis();
  deltaTime_odom = currentTime_odom - previousTime_odom;
  float dt = deltaTime_odom / 1000.0f;
  
  for (int i = 0; i < NUM_OF_MOTORS; i++){
    delta_encoder[i] = encoder_cnt[i] - encoder_last_cnt[i];
//    theta_dot[i] = delta_encoder[i] / (float) deltaTime_odo m / ENCODER_PPR * 2 * PI;
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
}
