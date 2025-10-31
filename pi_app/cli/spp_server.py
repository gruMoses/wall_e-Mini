#!/usr/bin/env python3
import signal
import sys
import json
import time
from pathlib import Path

try:
    import bluetooth  # pybluez
except Exception:
    print("PyBluez not installed. Install with: sudo apt-get install -y python3-bluez", file=sys.stderr)
    raise

try:
    # Local imports for protocol parsing (no auth)
    from pi_app.io.bt_proto import parse_cmd2, accept_cmd2, parse_v1, floats_to_bytes
except ModuleNotFoundError:
    sys.path.append(str(Path(__file__).resolve().parents[2]))
    from pi_app.io.bt_proto import parse_cmd2, accept_cmd2, parse_v1, floats_to_bytes  # type: ignore

SPP_UUID = "00001101-0000-1000-8000-00805F9B34FB"


def ints_to_bytes(left_i: int, right_i: int):
    """Convert integer values (-1000 to 1000) to byte values (0-255)"""
    DEAD_BAND_INT = 20
    TOP_SNAP_INT = 950
    
    def map_one(v: int) -> int:
        if v < -1000: v = -1000
        if v > 1000: v = 1000
        if v >= TOP_SNAP_INT: return 255
        if v <= -TOP_SNAP_INT: return 0
        if -DEAD_BAND_INT <= v <= DEAD_BAND_INT: return 126
        if v > 0:
            upper_span = 255 - 126
            mapped = 126 + int(round((v / 1000.0) * upper_span))
        else:
            lower_span = 126
            mapped = 126 - int(round((abs(v) / 1000.0) * lower_span))
        return max(0, min(255, mapped))
    
    return map_one(left_i), map_one(right_i)


def run_server() -> int:
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

                            # Write latest command to shared file for Wall-E app
                            try:
                                shared_data = {
                                    "left_byte": left_byte,
                                    "right_byte": right_byte,
                                    "last_update_epoch_s": time.time()
                                }
                                with open("/tmp/wall_e_bt_latest.json", "w") as sf:
                                    json.dump(shared_data, sf)
                                print(f"Wrote to shared file: L={left_byte} R={right_byte}", flush=True)
                            except Exception as e:
                                print(f"Error writing shared file: {e}", flush=True)
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

                            # Write latest command to shared file for Wall-E app
                            try:
                                shared_data = {
                                    "left_byte": left_byte,
                                    "right_byte": right_byte,
                                    "last_update_epoch_s": time.time()
                                }
                                with open("/tmp/wall_e_bt_latest.json", "w") as sf:
                                    json.dump(shared_data, sf)
                                print(f"V1 Wrote to shared file: L={left_byte} R={right_byte}", flush=True)
                            except Exception as e:
                                print(f"V1 Error writing shared file: {e}", flush=True)
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


