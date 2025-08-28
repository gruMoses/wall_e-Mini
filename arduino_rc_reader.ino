

#include <PinChangeInterrupt.h>
#define DEBUG 0

const byte NUM_CHANNELS = 4;
const byte chPins[NUM_CHANNELS] = {2, 3, 4, 6}; // Ch1, Ch2, Ch3, Ch5
volatile unsigned long pulseWidths[NUM_CHANNELS] = {1500, 1500, 1500, 1500};
volatile unsigned long pulseStart[NUM_CHANNELS] = {0, 0, 0, 0};

// Model X motor driver pin mapping (per attached photo)
// Left motor: ENB(PWM)=D5, IN3=D7, IN4=D8
// Right motor: ENA(PWM)=D9, IN2=D11, IN1=D12
const byte ENB_L = 5;   // Left speed PWM
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

void isrCH1() {
  if (digitalRead(chPins[0])) 
    pulseStart[0] = micros();
  else 
    pulseWidths[0] = micros() - pulseStart[0];
}
void isrCH2() {
  if (digitalRead(chPins[1])) 
    pulseStart[1] = micros();
  else 
    pulseWidths[1] = micros() - pulseStart[1];
}
void isrCH3() {
  if (digitalRead(chPins[2])) 
    pulseStart[2] = micros();
  else 
    pulseWidths[2] = micros() - pulseStart[2];
}
void isrCH4() {
  if (digitalRead(chPins[3])) 
    pulseStart[3] = micros();
  else 
    pulseWidths[3] = micros() - pulseStart[3];
}

void setup() {
  Serial.begin(115200);

  for (byte i = 0; i < NUM_CHANNELS; i++) {
    pinMode(chPins[i], INPUT_PULLUP);
  }
  attachPCINT(digitalPinToPCINT(chPins[0]), isrCH1, CHANGE);
  attachPCINT(digitalPinToPCINT(chPins[1]), isrCH2, CHANGE);
  attachPCINT(digitalPinToPCINT(chPins[2]), isrCH3, CHANGE);
  attachPCINT(digitalPinToPCINT(chPins[3]), isrCH4, CHANGE);

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
  while (Serial.available()) {
    char c = (char)Serial.read();
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
    }
  }
}

void loop() {
  static unsigned long nextCsvAt = 0;
  static unsigned long nextDbgAt = 0;
  const unsigned long nowMs = millis();

  // 1) Read current raw widths atomically once
  unsigned long r0, r1, r2, r3;
  noInterrupts();
  r0 = pulseWidths[0];
  r1 = pulseWidths[1];
  r2 = pulseWidths[2];
  r3 = pulseWidths[3];
  interrupts();

  // 2) Sanitize locally (bounds widened per receiver behavior)
  auto clamp = [](unsigned long v) { return (v < 800 || v > 2200) ? 1500UL : v; };
  unsigned long f0 = clamp(r0);
  unsigned long f1 = clamp(r1);
  unsigned long f2 = clamp(r2);
  unsigned long f3 = clamp(r3);

  // 3) Output CSV at ~50 Hz (20 ms)
  if (nowMs >= nextCsvAt) {
    Serial.print(f0); Serial.print(',');
    Serial.print(f1); Serial.print(',');
    Serial.print(f2); Serial.print(',');
    Serial.println(f3);
    nextCsvAt = nowMs + 20;
  }

  // 4) Optional debug output at ~2 Hz
#if DEBUG
  if (nowMs >= nextDbgAt) {
    Serial.print("DBG raw:");
    Serial.print(r0); Serial.print(',');
    Serial.print(r1); Serial.print(',');
    Serial.print(r2); Serial.print(',');
    Serial.print(r3);
    Serial.print(" filtered:");
    Serial.print(f0); Serial.print(',');
    Serial.print(f1); Serial.print(',');
    Serial.print(f2); Serial.print(',');
    Serial.println(f3);
    nextDbgAt = nowMs + 500;
  }
#endif

  // 5) Handle incoming motor commands from Pi every pass
  processSerialCommand();

  // Small pause to avoid hot spinning while keeping RX responsive
  delay(1);
}