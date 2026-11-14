

#include <PinChangeInterrupt.h>
#include "rc_channel.h"
#define DEBUG 0

// Line to the Pi, every 20 ms at 115200 baud (docs/rc_arduino_gps_wiring.md,
// section 7): ch1,ch2,ch3,ch4,ch5,ch6*HH in us, HH = CRC-8 of the text
// before '*'. A channel sends the width of its last valid pulse (500 to
// 2500 us) until 100 ms after that pulse ended, then 0 (RC_NO_SIGNAL_US,
// rc_channel.h); an out-of-range pulse is ignored. The Pi must accept this
// format first (requirement S12): pi_app/hardware/arduino_rc.py.
//
// Ch1..Ch5 on D2..D6. Ch6 on A0 (Kevin, 2026-11-13: wired now, not used by
// the Pi yet). A0 stays an input. Do not use D7..D12: the Model X code
// below drives them as outputs.
const byte NUM_CHANNELS = 6;
const byte chPins[NUM_CHANNELS] = {2, 3, 4, 5, 6, A0};
volatile RcPulseShared pulses[NUM_CHANNELS];  // written by the ISRs
RcEdgeState edges[NUM_CHANNELS];              // ISRs only
RcChannelState chState[NUM_CHANNELS];         // loop() only
const unsigned long CSV_PERIOD_MS = 20;

// Model X motor driver pin mapping (per attached photo)
// Left motor: ENB(PWM)=D10, IN3=D7, IN4=D8
// Right motor: ENA(PWM)=D9, IN2=D11, IN1=D12
const byte ENB_L = 10;  // Left speed PWM
const byte IN3_L = 7;   // Left dir 1
const byte IN4_L = 8;   // Left dir 2
const byte ENA_R = 9;   // Right speed PWM
const byte IN2_R = 11;  // Right dir 2
const byte IN1_R = 12;  // Right dir 1

// Track command neutral and bounds (match Pi side 0..255 with 126 center)
const int TRACK_MIN = 0;
const int TRACK_MAX = 255;
const int TRACK_CENTER = 126;

// Simple serial command buffer
char cmdBuf[32];
byte cmdLen = 0;
unsigned long lastByteTime = 0;
const unsigned long CMD_TIMEOUT_MS = 100;  // reset buffer if idle >100ms

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

  // Motor driver outputs
  pinMode(ENB_L, OUTPUT);
  pinMode(IN3_L, OUTPUT);
  pinMode(IN4_L, OUTPUT);
  pinMode(ENA_R, OUTPUT);
  pinMode(IN2_R, OUTPUT);
  pinMode(IN1_R, OUTPUT);
  // Ensure motors are stopped
  analogWrite(ENB_L, 0);
  analogWrite(ENA_R, 0);
  digitalWrite(IN3_L, LOW);
  digitalWrite(IN4_L, LOW);
  digitalWrite(IN2_R, LOW);
  digitalWrite(IN1_R, LOW);

  delay(1000);
  Serial.println("RC Reader initialized");
}

void driveLeft(int trackByte) {
  if (trackByte < TRACK_MIN) trackByte = TRACK_MIN;
  if (trackByte > TRACK_MAX) trackByte = TRACK_MAX;
  int delta = trackByte - TRACK_CENTER;
  if (delta == 0) {
    analogWrite(ENB_L, 0);
    digitalWrite(IN3_L, LOW);
    digitalWrite(IN4_L, LOW);
    return;
  }
  bool forward = delta > 0;
  int pwm = abs(delta) * 255 / (TRACK_MAX - TRACK_CENTER); // scale 0..129 -> 0..255
  if (pwm > 254) pwm = 254;
  digitalWrite(IN3_L, forward ? HIGH : LOW);
  digitalWrite(IN4_L, forward ? LOW : HIGH);
  analogWrite(ENB_L, pwm);
}

void driveRight(int trackByte) {
  if (trackByte < TRACK_MIN) trackByte = TRACK_MIN;
  if (trackByte > TRACK_MAX) trackByte = TRACK_MAX;
  int delta = trackByte - TRACK_CENTER;
  if (delta == 0) {
    analogWrite(ENA_R, 0);
    digitalWrite(IN1_R, LOW);
    digitalWrite(IN2_R, LOW);
    return;
  }
  bool forward = delta > 0;
  int pwm = abs(delta) * 255 / (TRACK_MAX - TRACK_CENTER);
  if (pwm > 254) pwm = 254;
  // Note: IN1/IN2 polarity may need swap depending on wiring
  digitalWrite(IN1_R, forward ? HIGH : LOW);
  digitalWrite(IN2_R, forward ? LOW : HIGH);
  analogWrite(ENA_R, pwm);
}

void processSerialCommand() {
  const unsigned long now = millis();
  if (cmdLen > 0 && (now - lastByteTime) > CMD_TIMEOUT_MS) {
    // Timed out waiting for newline; clear partial command
    cmdLen = 0;
    cmdBuf[0] = '\0';
  }
  while (Serial.available()) {
    char c = (char)Serial.read();
    lastByteTime = millis();
    if (c == '\n' || c == '\r') {
      if (cmdLen > 0) {
        cmdBuf[cmdLen] = '\0';
        // Expect: M,<left>,<right>
        if (cmdBuf[0] == 'M' && cmdBuf[1] == ',') {
          // Parse two integers
          int left = TRACK_CENTER;
          int right = TRACK_CENTER;
          int idx = 2;
          left = atoi(&cmdBuf[idx]);
          // Advance to next comma
          while (cmdBuf[idx] && cmdBuf[idx] != ',') idx++;
          if (cmdBuf[idx] == ',') {
            idx++;
            right = atoi(&cmdBuf[idx]);
            driveLeft(left);
            driveRight(right);
          }
        }
        cmdLen = 0;
      }
    } else if (cmdLen < (sizeof(cmdBuf) - 1)) {
      cmdBuf[cmdLen++] = c;
    } else {
      // overflow, reset
      cmdLen = 0;
      cmdBuf[0] = '\0';
    }
  }
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

  // 5) Handle incoming motor commands from Pi every pass
  processSerialCommand();

  // Small pause to avoid hot spinning while keeping RX responsive
  delay(1);
}