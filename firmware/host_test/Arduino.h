// Minimal host-side mock of the Arduino/AVR API so the firmware can be
// compiled and exercised with g++ on a PC (syntax + logic test, not timing).
#pragma once
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <cstdlib>
typedef uint8_t byte; typedef bool boolean;
#define B11110000 0xF0
#define B00001111 0x0F
#define B11100000 0xE0
#define B00010000 0x10
#define B11101111 0xEF
#define B11011111 0xDF
#define B10111111 0xBF
#define B01111111 0x7F
#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT 0
#define A0 14
#define A3 17
#ifndef PI
#define PI 3.14159265358979f
#endif
extern uint8_t DDRD, DDRB, PORTD, PORTB, PINB, PCICR, PCMSK0, TCCR1A, TCCR1B, TIMSK1, TWBR;
extern uint16_t OCR1A, TCNT1;
enum {PCIE0=0,PCINT0=0,PCINT1=1,PCINT2=2,PCINT3=3,PCINT4=4,WGM12=3,CS12=2,OCIE1A=1};
extern uint32_t mock_us;
inline uint32_t micros(){ return mock_us += 50; }
inline uint32_t millis(){ return mock_us/1000; }
inline void delay(unsigned long ms){ mock_us += ms*1000; }
inline void delayMicroseconds(unsigned int us){ mock_us += us; }
inline void pinMode(int,int){} inline void digitalWrite(int,int){}
extern int mock_adc[20];
inline int analogRead(int p){ return mock_adc[p]; }
inline void noInterrupts(){} inline void interrupts(){}
template<class T,class L,class H> T constrain(T x,L a,H b){ return x<a?a:(x>b?b:x); }
#define ISR(v) void v(void)
struct SerialMock { void begin(long){} int availableForWrite(){return 64;}
  void print(const char*s){ fputs(s, stdout);} };
extern SerialMock Serial;
