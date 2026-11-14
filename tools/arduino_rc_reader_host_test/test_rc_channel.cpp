// Host test of arduino_rc_reader/rc_channel.h (safety findings 1 and 5,
// docs/rc_arduino_gps_wiring.md). No Arduino calls: the ISR edge handling
// and the loop's per-channel output are driven with made-up micros() values.
//
// Build and run (pi_app/tests/test_arduino_rc_reader_host.py does this):
//   c++ -std=gnu++11 -Wall -Wextra -Werror -I ../../arduino_rc_reader \
//       test_rc_channel.cpp -o /tmp/test_rc_channel && /tmp/test_rc_channel

#include <stdint.h>
#include <stdio.h>

#include "rc_channel.h"

static int g_failures = 0;
static int g_checks = 0;

#define CHECK_EQ(actual, expected)                                                     \
  do {                                                                                 \
    ++g_checks;                                                                        \
    const unsigned long a_ = (unsigned long)(actual);                                  \
    const unsigned long e_ = (unsigned long)(expected);                                \
    if (a_ != e_) {                                                                    \
      ++g_failures;                                                                    \
      fprintf(stderr, "%s:%d: %s == %lu, expected %lu\n", __FILE__, __LINE__, #actual, \
              a_, e_);                                                                 \
    }                                                                                  \
  } while (0)

// One channel: the ISR side and the loop side, as in the sketch.
struct Channel {
  RcEdgeState edge;
  volatile RcPulseShared shared;
  RcChannelState loop;

  Channel() {
    rcInitEdge(edge);
    rcInitShared(shared);
    rcInitChannel(loop);
  }
  void rise(uint32_t t) { rcOnEdge(edge, shared, true, t); }
  void fall(uint32_t t) { rcOnEdge(edge, shared, false, t); }
  void pulse(uint32_t start, uint32_t width) {
    rise(start);
    fall(start + width);
  }
  uint16_t poll(uint32_t now) {
    RcPulseShared snap;
    rcCopyPulse(snap, shared);
    return rcChannelOutput(loop, snap, now);
  }
};

static void test_no_pulse_since_reset_is_no_signal() {
  Channel c;
  CHECK_EQ(c.poll(0), RC_NO_SIGNAL_US);
  CHECK_EQ(c.poll(5000000UL), RC_NO_SIGNAL_US);
}

static void test_valid_pulse_then_timeout() {
  Channel c;
  c.pulse(1000, 1500);  // ends at 2500
  CHECK_EQ(c.poll(2500), 1500);
  CHECK_EQ(c.poll(2500 + 60000UL), 1500);           // S2: never less than 60 ms
  CHECK_EQ(c.poll(2500 + RC_TIMEOUT_US), 1500);     // exactly 100 ms: still valid
  CHECK_EQ(c.poll(2500 + RC_TIMEOUT_US + 1), RC_NO_SIGNAL_US);
  CHECK_EQ(c.poll(2500 + 10 * RC_TIMEOUT_US), RC_NO_SIGNAL_US);
  // The next pulse brings the channel back.
  c.pulse(2000000UL, 2004);
  CHECK_EQ(c.poll(2000000UL + 2004 + 10), 2004);
}

static void test_range_edges() {
  const uint32_t widths[] = {0, 1, 499, 500, 992, 1500, 2004, 2500, 2501, 3000, 65535};
  const uint16_t expected[] = {RC_NO_SIGNAL_US, RC_NO_SIGNAL_US, RC_NO_SIGNAL_US, 500, 992, 1500,
                               2004, 2500, RC_NO_SIGNAL_US, RC_NO_SIGNAL_US, RC_NO_SIGNAL_US};
  for (unsigned i = 0; i < sizeof(widths) / sizeof(widths[0]); ++i) {
    Channel c;
    c.pulse(10000, widths[i]);
    CHECK_EQ(c.poll(10000 + widths[i] + 100), expected[i]);
  }
}

static void test_out_of_range_is_no_signal_not_1500() {
  // Finding 5: a CH5 pulse above the old 2200 limit used to read 1500.
  Channel c;
  c.pulse(0, 2000);
  CHECK_EQ(c.poll(2100), 2000);
  c.pulse(20000, 2600);
  CHECK_EQ(c.poll(22700), RC_NO_SIGNAL_US);
  c.pulse(40000, 2000);
  CHECK_EQ(c.poll(42100), 2000);
}

static void test_pulse_longer_than_16_bits() {
  Channel c;
  c.pulse(0, 70000UL);  // a stuck-high line that comes back after 70 ms
  CHECK_EQ(c.poll(70001UL), RC_NO_SIGNAL_US);
}

static void test_fall_without_rise_is_not_a_pulse() {
  Channel c;
  c.fall(1700);  // reset came in the middle of a pulse
  CHECK_EQ(c.poll(1800), RC_NO_SIGNAL_US);
  CHECK_EQ((unsigned long)c.shared.seq, 0);
  c.fall(1900);  // a second fall, still no rise
  CHECK_EQ(c.poll(2000), RC_NO_SIGNAL_US);
  c.pulse(20000, 1500);
  CHECK_EQ(c.poll(21600), 1500);
}

static void test_stuck_high_times_out() {
  // A lead that comes off while high (INPUT_PULLUP keeps it high): a rise,
  // then never a fall. The previous pulse times out.
  Channel c;
  c.pulse(0, 1500);
  c.rise(20000);
  CHECK_EQ(c.poll(21000), 1500);
  CHECK_EQ(c.poll(1500 + RC_TIMEOUT_US + 1), RC_NO_SIGNAL_US);
}

static void test_width_across_the_micros_wrap() {
  Channel c;
  const uint32_t start = 0xFFFFFF00UL;  // 256 us before the wrap
  c.pulse(start, 1500);                 // ends at 0x000004DC after the wrap
  const uint32_t end = start + 1500;
  CHECK_EQ(end, 0x000004DCUL);
  CHECK_EQ(c.poll(end + 5), 1500);
  CHECK_EQ(c.poll(end + RC_TIMEOUT_US + 1), RC_NO_SIGNAL_US);
}

static void test_dead_channel_stays_no_signal_after_the_wrap() {
  // The age of a dead channel's last pulse, (now - end) mod 2^32, comes
  // back near 0 after 71.6 min. The latch must hold.
  Channel c;
  const uint32_t end = 5000;
  c.pulse(end - 1500, 1500);
  uint32_t now = end;
  for (int ms = 0; ms <= 150; ++ms) {  // the loop polls about every 1 ms
    now = end + (uint32_t)ms * 1000UL;
    const uint16_t v = c.poll(now);
    CHECK_EQ(v, ms <= 100 ? 1500 : RC_NO_SIGNAL_US);
  }
  // 2^32 us later, plus a little: a naive age check would read 50 us.
  const uint32_t wrapped = end + 50;
  CHECK_EQ(c.poll(wrapped), RC_NO_SIGNAL_US);
  CHECK_EQ(c.poll(wrapped + 1000), RC_NO_SIGNAL_US);
}

static void test_50hz_train_with_missed_frames() {
  Channel c;
  uint32_t t = 100000UL;
  uint32_t last_end = 0;
  uint32_t last_width = 0;
  int bad = 0;
  // A healthy 50 Hz train, polled each 1 ms after each pulse ends (the
  // sketch reads micros() after its copy, so a copied pulse has always
  // ended): always valid.
  for (int frame = 0; frame < 200; ++frame, t += 20000UL) {
    last_width = 1000 + (uint32_t)(frame % 11) * 100;
    c.pulse(t, last_width);
    last_end = t + last_width;
    for (uint32_t now = last_end; now < t + 20000UL; now += 1000UL) {
      if (c.poll(now) != last_width) ++bad;
    }
  }
  CHECK_EQ(bad, 0);
  // Two missed frames (a 60 ms gap): still valid.
  CHECK_EQ(c.poll(last_end + 60000UL), last_width);
  // Five missed frames (120 ms): no signal.
  CHECK_EQ(c.poll(last_end + 120000UL), RC_NO_SIGNAL_US);
}

static void test_333hz_digital_mode() {
  Channel c;
  uint32_t t = 0;
  for (int frame = 0; frame < 1000; ++frame, t += 3000UL) {
    const uint32_t w = 500 + (uint32_t)(frame % 21) * 100;  // 500..2500
    c.pulse(t, w);
    CHECK_EQ(c.poll(t + w + 200), w);
  }
}

static void test_sequence_wrap() {
  // More than 65536 pulses: the 16-bit counter wraps; the loop sees every
  // change because it polls between pulses.
  Channel c;
  uint32_t t = 0;
  int bad = 0;
  for (long n = 0; n < 70000L; ++n, t += 3000UL) {
    c.pulse(t, 1500);
    if (c.poll(t + 1600) != 1500) ++bad;
  }
  CHECK_EQ(bad, 0);
  CHECK_EQ((unsigned long)c.shared.seq, 70000UL & 0xFFFFUL);
}

static void test_timeout_constant() {
  CHECK_EQ(RC_TIMEOUT_US, 100000UL);
  CHECK_EQ(RC_TIMEOUT_US >= 60000UL, 1);  // S2
  CHECK_EQ(RC_NO_SIGNAL_US, 0);
  CHECK_EQ(RC_VALID_MIN_US, 500);
  CHECK_EQ(RC_VALID_MAX_US, 2500);
}

int main() {
  test_no_pulse_since_reset_is_no_signal();
  test_valid_pulse_then_timeout();
  test_range_edges();
  test_out_of_range_is_no_signal_not_1500();
  test_pulse_longer_than_16_bits();
  test_fall_without_rise_is_not_a_pulse();
  test_stuck_high_times_out();
  test_width_across_the_micros_wrap();
  test_dead_channel_stays_no_signal_after_the_wrap();
  test_50hz_train_with_missed_frames();
  test_333hz_digital_mode();
  test_sequence_wrap();
  test_timeout_constant();
  if (g_failures) {
    fprintf(stderr, "%d of %d checks FAILED\n", g_failures, g_checks);
    return 1;
  }
  printf("rc_channel host test: %d checks passed\n", g_checks);
  return 0;
}
