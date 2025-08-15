from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import bluetooth  # pybluez

from pi_app.io.bt_proto import parse_cmd2, accept_cmd2, parse_v1, floats_to_bytes


def _ints_to_bytes(left_i: int, right_i: int) -> Tuple[int, int]:
    def map_one(v: int) -> int:
        if v < -1000:
            v = -1000
        if v > 1000:
            v = 1000
        normalized = (v + 1000) / 2000.0
        b = int(round(normalized * 255.0))
        if b < 0:
            return 0
        if b > 255:
            return 255
        return b

    return map_one(left_i), map_one(right_i)


@dataclass
class BtLatest:
    left_byte: int = 126
    right_byte: int = 126
    last_update_epoch_s: float = 0.0


class BtCommandServer:
    """
    Minimal unauthenticated SPP server that accepts CMD2 and V1 messages and
    maintains the latest command as bytes plus a freshness timestamp.
    """

    SPP_UUID = "00001101-0000-1000-8000-00805F9B34FB"

    def __init__(self) -> None:
        self._latest = BtLatest()
        self._lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None
        self._should_run = threading.Event()

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._should_run.set()
        self._thread = threading.Thread(target=self._run, name="BtCommandServer", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._should_run.clear()
        if self._thread:
            self._thread.join(timeout=2.0)

    def get_latest_bytes(self) -> BtLatest:
        with self._lock:
            return BtLatest(
                left_byte=self._latest.left_byte,
                right_byte=self._latest.right_byte,
                last_update_epoch_s=self._latest.last_update_epoch_s,
            )

    def _update_latest(self, left_b: int, right_b: int) -> None:
        with self._lock:
            self._latest.left_byte = left_b
            self._latest.right_byte = right_b
            self._latest.last_update_epoch_s = time.time()

    def _run(self) -> None:
        server_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
        server_sock.bind(("", bluetooth.PORT_ANY))
        server_sock.listen(1)
        bluetooth.advertise_service(
            server_sock,
            "WALL-E Control",
            service_id=self.SPP_UUID,
            service_classes=[self.SPP_UUID, bluetooth.SERIAL_PORT_CLASS],
            profiles=[bluetooth.SERIAL_PORT_PROFILE],
        )

        try:
            while self._should_run.is_set():
                try:
                    client_sock, _client_info = server_sock.accept()
                except Exception:
                    continue
                try:
                    # Informational hello (nonce not used)
                    client_sock.send(f"SRV:HELLO ver=2 sn=demo_nonce\n".encode("utf-8"))
                    buf = ""
                    pending_cmd2 = ""
                    last_seq: Optional[int] = None
                    while self._should_run.is_set():
                        data = client_sock.recv(1024)
                        if not data:
                            break
                        buf += data.decode("utf-8", errors="ignore")
                        while "\n" in buf:
                            line, buf = buf.split("\n", 1)
                            line = line.strip()
                            if not line:
                                continue
                            # Reassemble fragmented CMD2 lines
                            if line.startswith("CMD2:"):
                                if pending_cmd2:
                                    pending_cmd2 += line
                                else:
                                    pending_cmd2 = line
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
                                ok, _ = accept_cmd2(cmd, last_seq)
                                if ok:
                                    last_seq = cmd.seq
                                    lb, rb = _ints_to_bytes(cmd.left_i, cmd.right_i)
                                    self._update_latest(lb, rb)
                                    client_sock.send(f"ACK2:{cmd.seq};ok\n".encode("utf-8"))
                                else:
                                    client_sock.send(f"NAK2:{cmd.seq};code=old_seq\n".encode("utf-8"))
                                continue

                            v1 = parse_v1(line_to_parse)
                            if v1 is not None:
                                lf, rf, seq = v1
                                lb, rb = floats_to_bytes(lf, rf)
                                last_seq = seq
                                self._update_latest(lb, rb)
                                client_sock.send(f"ACK2:{seq};ok\n".encode("utf-8"))
                                continue
                finally:
                    try:
                        client_sock.close()
                    except Exception:
                        pass
        finally:
            try:
                server_sock.close()
            except Exception:
                pass


