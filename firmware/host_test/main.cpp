#include "Arduino.h"
#include "Wire.h"
uint8_t DDRD, DDRB, PORTD, PORTB, PINB, PCICR, PCMSK0, TCCR1A, TCCR1B, TIMSK1, TWBR;
uint16_t OCR1A, TCNT1; uint32_t mock_us = 1; int mock_adc[20] = {0};
SerialMock Serial; WireMock Wire;
// Port D pins drop to LOW inside the busy loop: emulate by masking writes
#include "../flight_controller/flight_controller.ino"
void feed(uint16_t r,uint16_t p,uint16_t t,uint16_t y){
  // simulate one pulse frame on each channel through the ISR
  uint16_t w[4]={r,p,t,y};
  for(int ch=0;ch<4;ch++){ PINB |= (1<<ch); PCINT0_vect(); mock_us += w[ch]-50; PINB &= ~(1<<ch); PCINT0_vect(); }
}
int main(){
  mock_adc[A0] = 942; mock_adc[A3] = 300;
  setup();
  printf("calibrated, status=%d\n", status);
  for(int i=0;i<300;i++){ feed(1500,1500,1000,1000); loop(); }      // yaw left, throttle low
  printf("after yaw-left: status=%d\n", status);
  for(int i=0;i<300;i++){ feed(1500,1500,1000,1500); loop(); }      // yaw centre -> STARTED
  printf("after centre: status=%d\n", status);
  for(int i=0;i<300;i++){ feed(1600,1500,1400,1500); loop(); }      // roll right stick
  printf("roll stick: esc=%u %u %u %u sp_roll=%.1f\n", esc_pulse[0],esc_pulse[1],esc_pulse[2],esc_pulse[3], pid_set_points[ROLL]);
  // stop feeding -> fail-safe ramp
  int n=0; while(status==STARTED && n<5000){ mock_us += 4000; loop(); n++; }
  printf("failsafe=%d status=%d after %d loops (%.2f s)\n", failsafe, status, n, n*0.004);
  return 0;
}
