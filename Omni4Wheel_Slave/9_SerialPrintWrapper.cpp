#include <stdarg.h>
#include <cstdio> 
#include <Arduino.h>

void SerialPrint(const char* format, ...) {
  char buffer[256];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  Serial.println(buffer);
}
