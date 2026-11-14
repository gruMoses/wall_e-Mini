// Minimal Arduino declarations, enough for a host syntax check of
// arduino_rc_reader/arduino_rc_reader.ino (-fsyntax-only). Not an emulator:
// nothing here runs. It does not prove the sketch builds for the AVR; build
// it with the Arduino toolchain (arduino:avr:nano:cpu=atmega328old) before
// a flash.
#ifndef ARDUINO_HOST_STUB_H
#define ARDUINO_HOST_STUB_H

#include <stdint.h>
#include <stdlib.h>

typedef uint8_t byte;

#define HIGH 0x1
#define LOW 0x0
#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2

static const uint8_t A0 = 14;

void pinMode(uint8_t pin, uint8_t mode);
int digitalRead(uint8_t pin);
void digitalWrite(uint8_t pin, uint8_t val);
void analogWrite(uint8_t pin, int val);
unsigned long millis(void);
unsigned long micros(void);
void delay(unsigned long ms);
void noInterrupts(void);
void interrupts(void);

class HardwareSerialStub {
 public:
  void begin(unsigned long baud);
  int available(void);
  int read(void);
  size_t print(char c);
  size_t print(const char *s);
  size_t print(unsigned int n);
  size_t print(int n);
  size_t print(unsigned long n);
  size_t print(long n);
  size_t println(void);
  size_t println(const char *s);
  size_t println(unsigned long n);
};
extern HardwareSerialStub Serial;

#endif
