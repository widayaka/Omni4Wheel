void robot_boot_screen(){
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0,0); display.println(F("Dhurobotic"));
  display.setCursor(0,10); display.println(F("Omni 4 Wheel Platform"));
  display.setCursor(0,20); display.println(F("By: Parama Diptya W"));

  display.display();
  delay(2000);
  display.clearDisplay();
}

void robot_home_screen(){
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  switch(pilih){
    case 0: pilih = 1; break;
    case 1: display.setTextSize(1);
            display.setTextColor(SSD1306_WHITE);
            display.setCursor(0,0);   display.println(F(">"));
            display.setCursor(10,0);  display.println(F("Run Program"));
            display.setCursor(10,10);  display.println(F("Cek Encoder"));
            display.setCursor(10,20);  display.println(F("Cek Motor"));
            display.display(); break;
            
    case 2: display.setTextSize(1);
            display.setTextColor(SSD1306_WHITE);
            display.setCursor(0,10);   display.println(F(">"));
            display.setCursor(10,0);  display.println(F("Run Program"));
            display.setCursor(10,10);  display.println(F("Cek Encoder"));
            display.setCursor(10,20);  display.println(F("Cek Motor"));
            display.display(); break;
            
    case 3: display.setTextSize(1);
            display.setTextColor(SSD1306_WHITE);
            display.setCursor(0,20);   display.println(F(">"));
            display.setCursor(10,0);  display.println(F("Run Program"));
            display.setCursor(10,10);  display.println(F("Cek Encoder"));
            display.setCursor(10,20);  display.println(F("Cek Motor"));
            display.display(); break;
    case 4: pilih = 3; break;
  }
  if (digitalRead(PB_DOWN)==LOW)  {delay(200); display.clearDisplay(); pilih++;}
  if (digitalRead(PB_UP)==LOW)    {delay(200); display.clearDisplay(); pilih--;}
}
