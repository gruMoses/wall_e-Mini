// Declarations of the PinChangeInterrupt library calls the sketch uses, for
// the host syntax check only (see Arduino.h in this folder).
#ifndef PIN_CHANGE_INTERRUPT_HOST_STUB_H
#define PIN_CHANGE_INTERRUPT_HOST_STUB_H

#include <stdint.h>

#define CHANGE 1

uint8_t digitalPinToPCINT(uint8_t pin);
void attachPCINT(uint8_t pcintNum, void (*userFunc)(void), uint8_t mode);

#endif
