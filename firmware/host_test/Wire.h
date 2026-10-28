#pragma once
#include "Arduino.h"
struct WireMock {
  int16_t regs[7] = {0,0,4096,0,0,0,0}; int idx = 0;  // level, 1 g on Z
  void begin(){} void beginTransmission(int){} void write(int){} int endTransmission(){return 0;}
  void requestFrom(int,int){ idx = 0; } int available(){ return 14; }
  int read(){ int16_t v = regs[idx/2]; int b = (idx%2==0) ? ((v>>8)&0xFF) : (v&0xFF); idx++; return (int8_t)b; }
};
extern WireMock Wire;
