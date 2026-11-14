#include <PinChangeInterrupt.h>
#include "rc_channel.h"
#define DEBUG 0

// RC bridge: reads the six receiver channels and streams them to the Pi.
// Nothing else (the Model X motor code is removed, Kevin 2026-11-13).
//
// Line to the Pi, every 20 ms at 115200 baud (docs/rc_arduino_gps_wiring.md,
// section 7): ch1,ch2,ch3,ch4,ch5,ch6*HH in us, HH = CRC-8 of the text
// before '*'. A channel sends the width of its last valid pulse (500 to
// 2500 us) until 100 ms after that pulse ended, then 0 (RC_NO_SIGNAL_US,
// rc_channel.h); an out-of-range pulse is ignored. The Pi must accept this
// format first (requirement S12): pi_app/hardware/arduino_rc.py.
//
// Ch1..Ch5 on D2..D6. Ch6 on A0 (Kevin, 2026-11-13: wired now, not used by
// the Pi yet). A0 stays an input. The sketch does not touch D7..D12: they
// stay at the reset default (input, no pull-up).
const byte NUM_CHANNELS = 6;
const byte chPins[NUM_CHANNELS] = {2, 3, 4, 5, 6, A0};
volatile RcPulseShared pulses[NUM_CHANNELS];  // written by the ISRs
RcEdgeState edges[NUM_CHANNELS];              // ISRs only
RcChannelState chState[NUM_CHANNELS];         // loop() only
const unsigned long CSV_PERIOD_MS = 20;

// Pin level first, then micros(), on both edges, so the read delay cancels
// out of the width (the same order as the old ISRs).
static inline void onRcEdge(byte i) {
  const bool high = digitalRead(chPins[i]) == HIGH;
  const uint32_t nowUs = micros();
  rcOnEdge(edges[i], pulses[i], high, nowUs);
}
void isrCH1() { onRcEdge(0); }
void isrCH2() { onRcEdge(1); }
void isrCH3() { onRcEdge(2); }
void isrCH4() { onRcEdge(3); }
void isrCH5() { onRcEdge(4); }
void isrCH6() { onRcEdge(5); }

void setup() {
  Serial.begin(115200);

  for (byte i = 0; i < NUM_CHANNELS; i++) {
    rcInitShared(pulses[i]);
    rcInitEdge(edges[i]);
    rcInitChannel(chState[i]);
    pinMode(chPins[i], INPUT_PULLUP);  // a lead that is off reads high: no edges, so no signal
  }
  attachPCINT(digitalPinToPCINT(chPins[0]), isrCH1, CHANGE);
  attachPCINT(digitalPinToPCINT(chPins[1]), isrCH2, CHANGE);
  attachPCINT(digitalPinToPCINT(chPins[2]), isrCH3, CHANGE);
  attachPCINT(digitalPinToPCINT(chPins[3]), isrCH4, CHANGE);
  attachPCINT(digitalPinToPCINT(chPins[4]), isrCH5, CHANGE);
  attachPCINT(digitalPinToPCINT(chPins[5]), isrCH6, CHANGE);

  delay(1000);
  Serial.println("RC Reader initialized");
}

void loop() {
  static unsigned long lastCsvMs = 0;
  static unsigned long lastDbgMs = 0;
  static uint16_t out[NUM_CHANNELS];
  static RcPulseShared snap[NUM_CHANNELS];
  const unsigned long nowMs = millis();

  // 1) Every pass: copy the ISR values with interrupts off (multi-byte
  //    values), then read micros() so no pulse in the copy ended after it.
  //    Updating every pass (about 1 ms) lets the timeout latch long before
  //    the 71.6 min micros() wrap.
  uint32_t nowUs;
  noInterrupts();
  for (byte i = 0; i < NUM_CHANNELS; i++) {
    rcCopyPulse(snap[i], pulses[i]);
  }
  nowUs = micros();
  interrupts();

  // 2) Last valid width, or RC_NO_SIGNAL_US 100 ms after the last valid pulse.
  for (byte i = 0; i < NUM_CHANNELS; i++) {
    out[i] = rcChannelOutput(chState[i], snap[i], nowUs);
  }

  // 3) Output CSV at ~50 Hz (20 ms). The unsigned difference stays right
  //    across the 49.7-day millis() wrap (safety finding 7).
  //    The line ends in "*HH", a CRC-8 of the text before it (rc_channel.h).
  if ((unsigned long)(nowMs - lastCsvMs) >= CSV_PERIOD_MS) {
    static char line[RC_LINE_BUF_SIZE];
    rcFormatLine(line, out, NUM_CHANNELS);
    Serial.println(line);
    lastCsvMs = nowMs;
  }

  // 4) Optional debug output at ~2 Hz. The Pi drops these lines.
#if DEBUG
  if ((unsigned long)(nowMs - lastDbgMs) >= 500UL) {
    Serial.print("DBG width/seq/age_us:");
    for (byte i = 0; i < NUM_CHANNELS; i++) {
      Serial.print(' ');
      Serial.print(snap[i].widthUs); Serial.print('/');
      Serial.print(snap[i].seq); Serial.print('/');
      Serial.print((uint32_t)(nowUs - snap[i].endUs));
    }
    Serial.println();
    lastDbgMs = nowMs;
  }
#else
  (void)lastDbgMs;
#endif

  // 5) Discard anything the Pi sends. Its old Model X command
  //    ("M,<left>,<right>", sent only when no VESC is found on CAN) now does
  //    nothing.
  while (Serial.available() > 0) {
    Serial.read();
  }

  // Small pause to avoid hot spinning.
  delay(1);
}
