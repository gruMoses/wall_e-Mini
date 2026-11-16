#!/usr/bin/env python3
"""Shutdown final-stage probe (design test 10, 2026-11-15). Read-only for the robot: it only writes a log.
mode p25: after SIGTERM it stays for 25 s and then exits by itself.
mode q:   it ignores SIGTERM and writes one line each second until it is killed.
Each line is written with fsync, so it is on the disk when the power goes off."""
import os, signal, sys, time
mode, path = sys.argv[1], sys.argv[2]
f = open(path, "a")
def log(msg):
    f.write("%.2f %.2f %s\n" % (time.time(), time.clock_gettime(time.CLOCK_BOOTTIME), msg))
    f.flush(); os.fsync(f.fileno())
term_at = [None]
def on_term(signum, frame):
    if term_at[0] is None:
        term_at[0] = time.monotonic(); log("SIGTERM")
signal.signal(signal.SIGTERM, on_term)
for s in (signal.SIGHUP, signal.SIGINT): signal.signal(s, signal.SIG_IGN)
log("start mode=%s pid=%d" % (mode, os.getpid()))
last = 0.0
while True:
    time.sleep(0.25)
    now = time.monotonic()
    if term_at[0] is not None:
        el = now - term_at[0]
        if mode == "p25" and el >= 25.0:
            log("exit by itself %.1f s after SIGTERM" % el); sys.exit(0)
        if now - last >= 1.0:
            last = now; log("alive %.1f s after SIGTERM" % el)
