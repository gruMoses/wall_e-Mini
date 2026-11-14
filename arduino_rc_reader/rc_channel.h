// RC channel pulse logic for arduino_rc_reader.ino, kept free of Arduino
// calls so that tools/arduino_rc_reader_host_test/ can compile and test it
// on a host computer.
//
// Safety findings 1 and 5 (docs/rc_arduino_gps_wiring.md, section 9):
// - A channel reports RC_NO_SIGNAL_US when it has had no complete pulse for
//   RC_TIMEOUT_US (a dead signal lead, a receiver without power, a receiver
//   that stops its outputs). Before, it held its last width forever.
// - A channel reports RC_NO_SIGNAL_US while its last pulse is outside
//   RC_VALID_MIN_US..RC_VALID_MAX_US. Before, it reported 1500 us.
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
  uint16_t widthUs;  // width of the last complete pulse, 0xFFFF if longer
  uint32_t endUs;    // micros() at the falling edge that ended it
  uint16_t seq;      // +1 for each complete pulse
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
  bool havePulse;  // a complete pulse was seen since reset
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
  out.widthUs = (w > 0xFFFFUL) ? (uint16_t)0xFFFF : (uint16_t)w;
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
  if (st.widthUs < RC_VALID_MIN_US || st.widthUs > RC_VALID_MAX_US) {
    return RC_NO_SIGNAL_US;
  }
  return st.widthUs;
}

#endif  // RC_CHANNEL_H
