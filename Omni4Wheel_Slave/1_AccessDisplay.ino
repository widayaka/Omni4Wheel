void RobotBootScreen(){
  display.clearDisplay(); display.setTextSize(1); display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0); display.println(F("Dhurobotic - 2025"));
  display.setCursor(0,10); display.println(F("Omni 4 Wheel Robot"));
  display.setCursor(0,20); display.println(F("By: Parama Diptya W"));
  display.display(); delay(2000); display.clearDisplay();
}

void RobotHomeScreen(){
  sendDataEncoder1 = encoder_cnt[0]; sendDataEncoder2 = encoder_cnt[1];
  sendDataEncoder3 = encoder_cnt[2]; sendDataEncoder4 = encoder_cnt[3];

  if (PUSH_BUTTON_RUN_IS_PRESSED) {for (int i = 0; i < NUM_OF_MOTORS; i++) {encoder_cnt[i] = 0;}}
    
  dataPackage = (String) startDataIdentifier + sendDataEncoder1 + dataSeparator + 
                                               sendDataEncoder2 + dataSeparator + 
                                               sendDataEncoder3 + dataSeparator + 
                                               sendDataEncoder4 + stopDataIdentifier;                                               
  SlaveSerial2.println(String(dataPackage));                                               

  SerialDataReceived();
  SerialDataParsing();  
  SerialDataConversion(); 
  SerialDataReset();

  if      (SerialData[0] == 7) {menu = 7;} // Robot Hold Position
  else if (SerialData[0] == 8) {menu = 8;} // Robot Joystick Control
  else if (SerialData[0] == 9) {menu = 9;} // Robot Odometry
  else {menu = 0; DriveMotor(SerialData[1], SerialData[2], SerialData[3], SerialData[4]);}

  display.clearDisplay(); display.setTextSize(1); display.setTextColor(SSD1306_WHITE);
  
  switch(pilih){
    case 0: pilih = 1; break;
    
    case 1: display.setTextSize(1);
            display.setTextColor(SSD1306_WHITE);
            display.setCursor(0,0);   display.println(F(">"));
            display.setCursor(10,0);  display.println(F("Encoder Check"));
            display.setCursor(10,10);  display.println(F("Motor Check"));
            display.setCursor(10,20);  display.println(F("Encoder Set 0"));      
            display.display(); 
            while (PUSH_BUTTON_OK_IS_PRESSED) {delay(100); menu = 1;}
            break;
            
    case 2: display.setTextSize(1);
            display.setTextColor(SSD1306_WHITE);
            display.setCursor(0,10);   display.println(F(">"));
            display.setCursor(10,0);  display.println(F("Encoder Check"));
            display.setCursor(10,10);  display.println(F("Motor Check"));
            display.setCursor(10,20);  display.println(F("Encoder Set 0"));
            display.display(); 
            while (PUSH_BUTTON_OK_IS_PRESSED) {delay(100); menu = 2;}
            break;
            
    case 3: display.setTextSize(1);
            display.setTextColor(SSD1306_WHITE);
            display.setCursor(0,20);   display.println(F(">"));
            display.setCursor(10,0);  display.println(F("Encoder Check"));
            display.setCursor(10,10);  display.println(F("Motor Check"));
            display.setCursor(10,20);  display.println(F("Encoder Set 0"));
            display.display(); 
            
            if (PUSH_BUTTON_OK_IS_PRESSED) {
              delay(100); 
              for (int i = 0; i < NUM_OF_ENCODER; i++){
                encoder_cnt[i] = 0;
              }
            }
            break;
            
    case 4: pilih = 3; break;
  }
  if (PUSH_BUTTON_DOWN_IS_PRESSED)  {delay(200); display.clearDisplay(); pilih++;}
  if (PUSH_BUTTON_UP_IS_PRESSED)    {delay(200); display.clearDisplay(); pilih--;}
}

void RobotMenuEncoder(){
  display.clearDisplay(); display.setTextSize(1); display.setTextColor(SSD1306_WHITE);

  display.setCursor(0,0);   display.print("Encoder Data");
  
  display.setCursor(0,10);  display.print("EN1");
  display.setCursor(20,10); display.print(":");
  display.setCursor(25,10); display.print(encoder_cnt[0]);

  display.setCursor(70,10);  display.print("EN2");
  display.setCursor(90,10); display.print(":");
  display.setCursor(95,10); display.print(encoder_cnt[1]);

  display.setCursor(0,20);  display.print("EN3");
  display.setCursor(20,20); display.print(":");
  display.setCursor(25,20); display.print(encoder_cnt[2]);

  display.setCursor(70,20);  display.print("EN4");
  display.setCursor(90,20); display.print(":");
  display.setCursor(95,20); display.print(encoder_cnt[3]);

  display.display();

  while (PUSH_BUTTON_RUN_IS_PRESSED) {
    for (int i = 0; i < NUM_OF_ENCODER; i++){
      encoder_cnt[i] = 0;  
    }
  } 
  while (PUSH_BUTTON_OK_IS_PRESSED) {delay(100); menu = 0;}
}

void RobotMenuMotor(){
  display.clearDisplay();   display.setTextSize(1); display.setTextColor(SSD1306_WHITE);
  
  display.setCursor(0,0);   display.print("Warning!");
  display.setCursor(0,10);  display.print("Motor is running...");
  display.display();

  int start = 0;
  while (start < 2){
    DriveMotor(50,50,50,50); delay(1000);
    DriveMotor(-50,-50,-50,-50); delay(1000);
    start++;
  }
  
  DriveMotor(0,0,0,0);  
  display.clearDisplay(); display.setTextSize(1); display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);   display.print("Checking finished!");
  display.setCursor(0,10);  display.print("Exitting...");
  display.display();
  delay(2000);
  menu = 0;
}
