// RC channel pulse logic for arduino_rc_reader.ino, kept free of Arduino
// calls so that tools/arduino_rc_reader_host_test/ can compile and test it
// on a host computer.
//
// Safety findings 1 and 5 (docs/rc_arduino_gps_wiring.md, section 9). One
// rule: a channel reports the width of its last VALID pulse (RC_VALID_MIN_US
// to RC_VALID_MAX_US) until RC_TIMEOUT_US after that pulse ended, then
// RC_NO_SIGNAL_US; before its first valid pulse it reports RC_NO_SIGNAL_US.
// - A dead signal lead, a receiver without power or a receiver that stops
//   its outputs thus reads no signal after 100 ms. Before, the channel held
//   its last width forever.
// - An out-of-range pulse is ignored (hold last good, Kevin 2026-11-13), and
//   a stream of them reads no signal after 100 ms. Before, it read 1500 us.
//
// All types have explicit widths: int is 16 bits on the AVR and 32 bits on
// the host, so the host test would not see an AVR overflow otherwise.
#ifndef RC_CHANNEL_H
#define RC_CHANNEL_H

#include <stdint.h>

// The value sent for "no signal / invalid". The Pi (pi_app/control/
// rc_signal.py) reads it as no signal: never high, low, mid-band or neutral.
const uint16_t RC_NO_SIGNAL_US = 0;

// Valid widths: requirement S1, both HotRC F-06A output modes (manual
// page 6): 1000-2500 us at 50 Hz, 500-2500 us at 333 Hz. The logs show
// 992-2004 us. A narrower range would report a valid digital-mode pulse as
// no signal; a wider one would pass garbage that no receiver mode sends.
const uint16_t RC_VALID_MIN_US = 500;
const uint16_t RC_VALID_MAX_US = 2500;

// No complete pulse for this long: no signal. Requirement S2: 100 ms, five
// 50 Hz frames; never less than 60 ms (three frames).
const uint32_t RC_TIMEOUT_US = 100000UL;

// Written by the pin-change ISR only. The loop copies it with interrupts
// off (multi-byte values), see rcCopyPulse().
struct RcPulseShared {
  uint16_t widthUs;  // width of the last VALID pulse (rcOnEdge drops the others)
  uint32_t endUs;    // micros() at the falling edge that ended that pulse
  uint16_t seq;      // +1 for each valid pulse
};

// ISR-only edge state.
struct RcEdgeState {
  uint32_t riseUs;
  bool risen;  // a rising edge was seen and its falling edge was not yet
};

// Loop-only state.
struct RcChannelState {
  uint16_t lastSeq;
  uint16_t widthUs;
  uint32_t endUs;
  bool havePulse;  // a valid pulse was seen since reset
  bool timedOut;   // latched until the next pulse, so the micros() wrap
                   // (71.6 min) cannot make an old pulse look fresh again
};

inline void rcInitShared(volatile RcPulseShared &p) {
  p.widthUs = 0;
  p.endUs = 0;
  p.seq = 0;
}

inline void rcInitEdge(RcEdgeState &e) {
  e.riseUs = 0;
  e.risen = false;
}

inline void rcInitChannel(RcChannelState &st) {
  st.lastSeq = 0;
  st.widthUs = 0;
  st.endUs = 0;
  st.havePulse = false;
  st.timedOut = false;
}

// Called from the pin-change ISR with the pin level after the edge and
// micros(). A falling edge with no rising edge before it (the reset came in
// the middle of a pulse) is not a pulse.
inline void rcOnEdge(RcEdgeState &e, volatile RcPulseShared &out, bool levelHigh, uint32_t nowUs) {
  if (levelHigh) {
    e.riseUs = nowUs;
    e.risen = true;
    return;
  }
  if (!e.risen) {
    return;
  }
  e.risen = false;
  const uint32_t w = nowUs - e.riseUs;  // unsigned: right across the micros() wrap
  // Hold last good (Kevin, 2026-11-13): an out-of-range pulse changes
  // nothing, neither the width nor the time of the last valid pulse. So one
  // glitch is ignored, and a stream of bad pulses reads as no signal 100 ms
  // after the last valid one.
  if (w < RC_VALID_MIN_US || w > RC_VALID_MAX_US) {
    return;
  }
  out.widthUs = (uint16_t)w;
  out.endUs = nowUs;
  out.seq = (uint16_t)(out.seq + 1);
}

// Plain copy of the shared values. Call it with interrupts off.
inline void rcCopyPulse(RcPulseShared &dst, const volatile RcPulseShared &src) {
  dst.widthUs = src.widthUs;
  dst.endUs = src.endUs;
  dst.seq = src.seq;
}

// The value to send for one channel. snap is a copy taken with interrupts
// off; nowUs is micros() read after that copy, so snap.endUs is not later
// than nowUs.
inline uint16_t rcChannelOutput(RcChannelState &st, const RcPulseShared &snap, uint32_t nowUs) {
  if (snap.seq != st.lastSeq) {
    st.lastSeq = snap.seq;
    st.widthUs = snap.widthUs;
    st.endUs = snap.endUs;
    st.havePulse = true;
    st.timedOut = false;
  }
  if (!st.havePulse) {
    return RC_NO_SIGNAL_US;
  }
  if (!st.timedOut && (uint32_t)(nowUs - st.endUs) > RC_TIMEOUT_US) {
    st.timedOut = true;
  }
  if (st.timedOut) {
    return RC_NO_SIGNAL_US;
  }
  // A second guard only: rcOnEdge publishes valid widths only.
  if (st.widthUs < RC_VALID_MIN_US || st.widthUs > RC_VALID_MAX_US) {
    return RC_NO_SIGNAL_US;
  }
  return st.widthUs;
}

// CRC-8/SMBUS (poly 0x07, init 0): the same as rc_line_crc8() in
// pi_app/control/rc_signal.py. Check value: "123456789" -> 0xF4. The Pi
// needs it because the 500..2500 us range lets a dropped byte read as a
// valid width ("1500" without its "1" is 500 us).
inline uint8_t rcCrc8Update(uint8_t crc, uint8_t data) {
  crc ^= data;
  for (uint8_t bit = 0; bit < 8; bit++) {
    crc = (crc & 0x80) ? (uint8_t)((uint8_t)(crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
  }
  return crc;
}

// Size for rcFormatLine(): 6 fields of at most 5 digits, 5 commas, "*HH",
// and the NUL (6 * 6 + 3 = 39).
const uint8_t RC_LINE_BUF_SIZE = 6 * 5 + 5 + 3 + 1;

// Writes "v0,v1,...,v(count-1)*HH" and a NUL into buf, where HH is the
// CRC-8 of the text before '*' in upper-case hex. Returns the length
// without the NUL. buf needs 6 * count + 3 bytes with the NUL
// (RC_LINE_BUF_SIZE for count 6).
inline uint8_t rcFormatLine(char *buf, const uint16_t *vals, uint8_t count) {
  static const char hex[] = "0123456789ABCDEF";
  uint8_t pos = 0;
  for (uint8_t i = 0; i < count; i++) {
    if (i > 0) {
      buf[pos++] = ',';
    }
    char digits[5];
    uint8_t n = 0;
    uint16_t v = vals[i];
    do {
      digits[n++] = (char)('0' + (v % 10));
      v = (uint16_t)(v / 10);
    } while (v != 0 && n < 5);
    while (n > 0) {
      buf[pos++] = digits[--n];
    }
  }
  uint8_t crc = 0;
  for (uint8_t j = 0; j < pos; j++) {
    crc = rcCrc8Update(crc, (uint8_t)buf[j]);
  }
  buf[pos++] = '*';
  buf[pos++] = hex[crc >> 4];
  buf[pos++] = hex[crc & 0x0F];
  buf[pos] = '\0';
  return pos;
}

#endif  // RC_CHANNEL_H
