#!/usr/bin/env python3
import os
import signal
import sys
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

                    cmd = parse_cmd2(line_to_parse)
                    if cmd is not None:
                        ok, reason = accept_cmd2(cmd, last_seq)
                        if ok:
                            last_seq = cmd.seq
                            client_sock.send(f"ACK2:{cmd.seq};ok\n".encode("utf-8"))
                            print(f"OK  seq={cmd.seq} left_i={cmd.left_i} right_i={cmd.right_i}")
                            try:
                                with open(log_path, "a", buffering=1) as lf:
                                    lf.write(f"CMD2 ok seq={cmd.seq} left={cmd.left_i} right={cmd.right_i}\n")
                            except Exception:
                                pass
                        else:
                            client_sock.send(f"NAK2:{cmd.seq};code={reason}\n".encode("utf-8"))
                            print(f"NAK seq={cmd.seq} code={reason} line={line_to_parse}")
                            try:
                                with open(log_path, "a", buffering=1) as lf:
                                    lf.write(f"CMD2 nak seq={cmd.seq} code={reason}\n")
                            except Exception:
                                pass
                        continue
                    v1 = parse_v1(line_to_parse)
                    if v1 is not None:
                        lf, rf, seq = v1
                        lb, rb = floats_to_bytes(lf, rf)
                        last_seq = seq
                        client_sock.send(f"ACK2:{seq};ok\n".encode("utf-8"))
                        print(f"V1  seq={seq} left_f={lf:.3f} right_f={rf:.3f} -> bytes {lb},{rb}")
                        try:
                            with open(log_path, "a", buffering=1) as lfout:
                                lfout.write(f"V1 ok seq={seq} left_f={lf:.3f} right_f={rf:.3f} bytes={lb},{rb}\n")
                        except Exception:
                            pass
                        continue
                    print(f"RX  {line}")
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


