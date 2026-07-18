#!/usr/bin/env python3
import os
import signal
import sys
import json
import time
from pathlib import Path

# NOTE: ``import bluetooth`` (pybluez) is deliberately deferred into
# ``run_server()`` so this module stays importable on dev/CI hosts without
# pybluez. That lets the control-output gate below (and its tests) be exercised
# without a Bluetooth stack.

try:
    from pi_app.io.bt_proto import parse_cmd2, accept_cmd2, parse_v1, floats_to_bytes
    from pi_app.control.mapping import CENTER_OUTPUT_VALUE, MAX_OUTPUT, MIN_OUTPUT
except ModuleNotFoundError:
    sys.path.append(str(Path(__file__).resolve().parents[2]))
    from pi_app.io.bt_proto import parse_cmd2, accept_cmd2, parse_v1, floats_to_bytes  # type: ignore
    from pi_app.control.mapping import CENTER_OUTPUT_VALUE, MAX_OUTPUT, MIN_OUTPUT  # type: ignore

SPP_UUID = "00001101-0000-1000-8000-00805F9B34FB"

# Shared override file that ``pi_app/app/main.py`` polls on a 600 ms freshness
# window and feeds straight into ``controller.process(bt_override_bytes=...)``
# (main.py:435,527 -> controller.py:827). Writing here is equivalent to
# commanding the tracks whenever RC has the robot armed.
SHARED_OVERRIDE_PATH = "/tmp/wall_e_bt_latest.json"

# Opt-in gate for the SPP -> motor-override channel. Default OFF.
#
# With the gate OFF (the default) the SPP server still accepts connections and
# parses / ACKs / logs commands for bench diagnostics, but it does NOT write the
# shared override file — so Bluetooth SPP data cannot reach
# ``controller.process`` or the motors. This retires the historical
# "uncoordinated third writer" of the override file: the RC transmitter (via the
# Arduino, the safety authority) and the session-gated web teleop
# (``pi_app/web/teleop.py`` — arming ceremony, latched e-stop, 250 ms deadman,
# speed cap, stale guard) remain the only two writers that can command motion.
#
# Set ``WALL_E_SPP_CONTROL_ENABLED=1`` to re-enable the legacy Android/BT
# bench-drive path. That path bypasses the entire web-teleop session-safety
# layer and is gated only by RC arming + the 600 ms file-freshness fallback, so
# enable it only for supervised bench testing.
SPP_CONTROL_ENV_VAR = "WALL_E_SPP_CONTROL_ENABLED"
_TRUTHY = {"1", "true", "yes", "on"}


def control_output_enabled() -> bool:
    """Return True only if the SPP -> motor-override channel is explicitly enabled."""
    return os.environ.get(SPP_CONTROL_ENV_VAR, "").strip().lower() in _TRUTHY


def write_control_override(left_byte: int, right_byte: int, *,
                           path: str = SHARED_OVERRIDE_PATH,
                           now=time.time) -> bool:
    """Write the shared motor-override file consumed by ``main.py``.

    Returns ``True`` if the file was written. Returns ``False`` — writing
    nothing — when the SPP control channel is disabled (the default), which is
    the mechanism that keeps Bluetooth SPP data from reaching the motors. Write
    errors are swallowed and reported as ``False``.
    """
    if not control_output_enabled():
        return False
    try:
        with open(path, "w") as sf:
            json.dump({
                "left_byte": int(left_byte),
                "right_byte": int(right_byte),
                "last_update_epoch_s": now(),
            }, sf)
        return True
    except Exception as e:
        print(f"Error writing shared file: {e}", flush=True)
        return False


def ints_to_bytes(left_i: int, right_i: int):
    DEAD_BAND_INT = 20
    TOP_SNAP_INT = 950

    def map_one(v: int) -> int:
        if v < -1000: v = -1000
        if v > 1000: v = 1000
        if v >= TOP_SNAP_INT: return MAX_OUTPUT
        if v <= -TOP_SNAP_INT: return MIN_OUTPUT
        if -DEAD_BAND_INT <= v <= DEAD_BAND_INT: return CENTER_OUTPUT_VALUE
        if v > 0:
            upper_span = MAX_OUTPUT - CENTER_OUTPUT_VALUE
            mapped = CENTER_OUTPUT_VALUE + int(round((v / 1000.0) * upper_span))
        else:
            lower_span = CENTER_OUTPUT_VALUE
            mapped = CENTER_OUTPUT_VALUE - int(round((abs(v) / 1000.0) * lower_span))
        return max(MIN_OUTPUT, min(MAX_OUTPUT, mapped))

    return map_one(left_i), map_one(right_i)


def run_server() -> int:
    try:
        import bluetooth  # pybluez
    except Exception:
        print("PyBluez not installed. Install with: sudo apt-get install -y python3-bluez", file=sys.stderr)
        raise

    if control_output_enabled():
        print(f"SPP control output ENABLED ({SPP_CONTROL_ENV_VAR}=1) — "
              f"Bluetooth commands will drive the motors when RC is armed", flush=True)
    else:
        print(f"SPP control output DISABLED (default; set {SPP_CONTROL_ENV_VAR}=1 to enable) — "
              f"commands are parsed/ACKed/logged but will NOT reach the motors", flush=True)

    server_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    server_sock.bind(("", bluetooth.PORT_ANY))
    server_sock.listen(1)
    port = server_sock.getsockname()[1]
    bluetooth.advertise_service(
        server_sock,
        "WALL-E Control",
        service_id=SPP_UUID,
        service_classes=[SPP_UUID, bluetooth.SERIAL_PORT_CLASS],
        profiles=[bluetooth.SERIAL_PORT_PROFILE],
    )
    print(f"SPP server listening on RFCOMM channel {port} (UUID {SPP_UUID})")

    # No secret/nonce needed in unauthenticated mode
    expected_nonce = "demo_nonce"
    log_path = "/home/pi/bt_traffic.log"

    should_run = True

    def on_sigint(_s, _f):
        nonlocal should_run
        should_run = False

    signal.signal(signal.SIGINT, on_sigint)

    while should_run:
        try:
            client_sock, client_info = server_sock.accept()
        except Exception:
            continue
        print(f"Client connected: {client_info}")
        try:
            with open(log_path, "a", buffering=1) as lf:
                lf.write(f"client_connected {client_info}\n")
        except Exception:
            pass
        last_seq = None
        pending_cmd2: str = ""
        try:
            # Try V2 protocol first (with nonce)
            client_sock.send(f"SRV:HELLO ver=2 sn={expected_nonce}\n".encode("utf-8"))
            buf = ""
            while should_run:
                data = client_sock.recv(1024)
                if not data:
                    break
                buf += data.decode("utf-8", errors="ignore")
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    line = line.strip()
                    if not line:
                        continue
                    # Reassemble fragmented CMD2 lines that span newlines
                    if line.startswith("CMD2:"):
                        if pending_cmd2:
                            pending_cmd2 += line
                        else:
                            pending_cmd2 = line
                        # We consider a CMD2 complete when it has at least 5 semicolons after the prefix
                        # (left;right;seq;ts;nonce;hmac)
                        if pending_cmd2.count(";") < 5:
                            continue
                        line_to_parse = pending_cmd2
                        pending_cmd2 = ""
                    else:
                        if pending_cmd2:
                            pending_cmd2 += line
                            if pending_cmd2.count(";") < 5:
                                continue
                            line_to_parse = pending_cmd2
                            pending_cmd2 = ""
                        else:
                            line_to_parse = line

                    # Try V2 protocol first
                    cmd = parse_cmd2(line_to_parse)
                    if cmd is not None:
                        ok, reason = accept_cmd2(cmd, last_seq)
                        if ok:
                            last_seq = cmd.seq
                            client_sock.send(f"ACK2:{cmd.seq};ok\n".encode("utf-8"))
                            print(f"V2 OK  seq={cmd.seq} left_i={cmd.left_i} right_i={cmd.right_i}", flush=True)

                            # Convert to bytes for Wall-E app compatibility
                            left_byte, right_byte = ints_to_bytes(cmd.left_i, cmd.right_i)
                            print(f"V2 DEBUG: Converted {cmd.left_i},{cmd.right_i} -> {left_byte},{right_byte}", flush=True)

                            # Forward to the shared motor-override file ONLY when
                            # the SPP control channel is explicitly enabled
                            # (default OFF). See write_control_override /
                            # SPP_CONTROL_ENV_VAR above: this used to be an
                            # uncoordinated THIRD writer of the override file
                            # (bypassing the web-teleop session-safety layer);
                            # it is now retired to a supervised, opt-in
                            # bench-test path.
                            if write_control_override(left_byte, right_byte):
                                print(f"Wrote to shared file: L={left_byte} R={right_byte}", flush=True)
                            else:
                                print(f"SPP control disabled ({SPP_CONTROL_ENV_VAR} unset) — "
                                      f"dropped L={left_byte} R={right_byte}", flush=True)
                        else:
                            client_sock.send(f"NAK2:{cmd.seq};code={reason}\n".encode("utf-8"))
                            print(f"V2 NAK seq={cmd.seq} reason={reason}", flush=True)
                    else:
                        # Try V1 protocol
                        v1_cmd = parse_v1(line_to_parse)
                        if v1_cmd is not None:
                            left_f, right_f, seq = v1_cmd
                            print(f"V1 OK  seq={seq} left_f={left_f} right_f={right_f}", flush=True)

                            # Convert floats to bytes for Wall-E app compatibility
                            left_byte, right_byte = floats_to_bytes(left_f, right_f)
                            print(f"V1 DEBUG: Converted {left_f},{right_f} -> {left_byte},{right_byte}", flush=True)

                            # Same default-OFF gate as the V2 path above.
                            if write_control_override(left_byte, right_byte):
                                print(f"V1 Wrote to shared file: L={left_byte} R={right_byte}", flush=True)
                            else:
                                print(f"V1 SPP control disabled ({SPP_CONTROL_ENV_VAR} unset) — "
                                      f"dropped L={left_byte} R={right_byte}", flush=True)
                        elif line_to_parse == "PING":
                            print("V1 PING received", flush=True)
                        elif line_to_parse.startswith("ARM:"):
                            print(f"V1 ARM command: {line_to_parse} (ignored for safety)", flush=True)
                        else:
                            print(f"Unknown command: {line_to_parse}", flush=True)

                    # Log successful commands
                    try:
                        with open(log_path, "a", buffering=1) as lf:
                            if cmd is not None and 'ok' in locals() and ok:
                                lf.write(f"CMD2 ok seq={cmd.seq} left={cmd.left_i} right={cmd.right_i}\n")
                            elif v1_cmd is not None:
                                left_f, right_f, seq = v1_cmd
                                lf.write(f"V1 ok seq={seq} left={left_f} right={right_f}\n")
                            elif line_to_parse == "PING":
                                lf.write("PING received\n")
                    except Exception:
                        pass
        except Exception as e:
            print(f"Client error: {e}")
        finally:
            try:
                client_sock.close()
            except Exception:
                pass
            print("Client disconnected")

    try:
        server_sock.close()
    except Exception:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(run_server())


