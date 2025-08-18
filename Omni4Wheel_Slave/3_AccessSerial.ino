//void SerialMonitorTest(){
//  while (Serial.available()>0){
//    dataReceived = Serial.read();
//    if (dataReceived == 'a'){digitalWrite(LED_BUILTIN, HIGH);}
//    else if (dataReceived == 'b'){digitalWrite(LED_BUILTIN, LOW);}
//  }
//}
//
//void MasterToSlaveSerialTest(){
//  if (SlaveSerial2.available()>0){
//    String message = SlaveSerial2.readStringUntil('\n');
//    Serial.println("Slave Received: " + message);
//  }
//}
//
//void SlaveToMasterSerialTest(){
//  SlaveSerial2.println(String(counter));
//  Serial.println("Slave Sent: " + String(counter));
//  counter++;
//  delay(100);
//}
//
//void SlaveToMasterMultipleDataSerialTest(){
//  data1 = 10;
//  data2 = 20;
//  data3 = 30;
//  data4 = 40;
//  
//  dataPackage = (String) startDataIdentifier + data1 + dataSeparator + 
//                                               data2 + dataSeparator + 
//                                               data3 + dataSeparator + 
//                                               data4 + stopDataIdentifier;
//  
//  SlaveSerial2.println(String(dataPackage));
//  Serial.println(String(dataPackage));
//  delay(100);
//}
//
//void sendEncoderDataSlaveToMaster(){
//  encoderAll_RPM();
//  
//  sendDataRPM1 = encoder_velocity[0]; 
//  sendDataRPM2 = encoder_velocity[1]; 
//  sendDataRPM3 = encoder_velocity[2]; 
//  sendDataRPM4 = encoder_velocity[3];
//  
//  sendDataEncoder1 = encoder_cnt[0];  
//  sendDataEncoder2 = encoder_cnt[1];  
//  sendDataEncoder3 = encoder_cnt[2];  
//  sendDataEncoder4 = encoder_cnt[3];
//  
//  dataPackage = (String) startDataIdentifier + sendDataEncoder1 + dataSeparator + 
//                                               sendDataEncoder2 + dataSeparator + 
//                                               sendDataEncoder3 + dataSeparator + 
//                                               sendDataEncoder4 + dataSeparator +
//                                               
//                                               sendDataRPM1 + dataSeparator + 
//                                               sendDataRPM2 + dataSeparator + 
//                                               sendDataRPM3 + dataSeparator + 
//                                               sendDataRPM4 + dataSeparator + stopDataIdentifier;
//                                               
//  SlaveSerial2.println(String(dataPackage));  
//}

//=====================================================================================================================================================//
void SerialDataReceived(){
  while (SlaveSerial2.available() > 0){                      //Open serial connection using MasterSerial2 (UART2)
    dataReceived = SlaveSerial2.read();                      //Read serial data and save the data to dataReceived variable   
    if (dataReceived == startDataIdentifier){                //Checking the data to read header '*' character
      bufferDataIn = startDataIdentifier;                    //Saving the data '*' character to bufferDataIn variable
      bufferDataIn += SlaveSerial2.readStringUntil('#');     //Updating the data in bufferDataIn variable until read '#' character 
      dataIsComplete = true;                                 //Change the boolean state of dataIsComplete variable
      bufferDataIn += stopDataIdentifier;                    //Updating the data in bufferDataIn variable until read '#' character
      break;                                                 //exit from while loop
    }
  }
}

void SerialDataParsing(){
  for (int i = 0; i < bufferDataIn.length(); i++){
    if (bufferDataIn[i] == startDataIdentifier){bufferDataParsing[indexOfData] = "";}
    else if (bufferDataIn[i] != dataSeparator){bufferDataParsing[indexOfData] += bufferDataIn[i];}
    else{indexOfData++; bufferDataParsing[indexOfData]="";}
  }
}

void SerialDataConversion(){
  for (int i = 0; i < 10; i++){
    SerialData[i] = bufferDataParsing[i].toInt();
  }
}

void SerialDataConversionOdometry(){
  SerialData[0] = bufferDataParsing[0].toInt();
  for (int i = 1; i < 10; i++){
    SerialData[i] = bufferDataParsing[i].toFloat();  
  }
}

void SerialDataShow(){
  Serial.print("Data1 = "); Serial.print(SerialData[0]); Serial.print("\t");
  Serial.print("Data2 = "); Serial.print(SerialData[1]); Serial.print("\t");
  Serial.print("Data3 = "); Serial.print(SerialData[2]); Serial.print("\t");
  Serial.print("Data4 = "); Serial.print(SerialData[3]); Serial.println("\t");
}

void SerialDataReset(){
  dataReceived = 0; bufferDataIn=""; dataIsComplete = false; indexOfData=0;
}
//=====================================================================================================================================================//
