import time
import sys
from pathlib import Path

try:
    from pi_app.hardware.vesc import VescCanDriver
    from pi_app.hardware.arduino_modelx import ArduinoModelXDriver
    from pi_app.hardware.arduino_rc import ArduinoRCReader
    from pi_app.control.controller import Controller, RCInputs
    from pi_app.io.bt_input import BtCommandServer
except ModuleNotFoundError:
    # Allow running as a script: python3 /home/pi/pi_app/app/main.py
    sys.path.append(str(Path(__file__).resolve().parents[2]))
    from pi_app.hardware.vesc import VescCanDriver  # type: ignore
    from pi_app.hardware.arduino_modelx import ArduinoModelXDriver  # type: ignore
    from pi_app.hardware.arduino_rc import ArduinoRCReader  # type: ignore
    from pi_app.control.controller import Controller, RCInputs  # type: ignore
    from pi_app.io.bt_input import BtCommandServer  # type: ignore


def run() -> None:
    rc_reader = ArduinoRCReader()
    port = rc_reader.start()
    print(f"Arduino RC detected on {port}")

    motor_driver = None
    if VescCanDriver.detect():
        print("VESC over CAN detected; using VESC driver")
        motor_driver = VescCanDriver()
    else:
        print("VESC not detected; using Arduino Model X motor driver (stub)")
        motor_driver = ArduinoModelXDriver(rc_reader=rc_reader)

    controller = Controller(motor_driver=motor_driver)
    bt_server = BtCommandServer()
    bt_server.start()

    # Debug trackers for BT byte extrema
    min_btL, max_btL = 255, 0
    min_btR, max_btR = 255, 0

    try:
        while True:
            s = rc_reader.get_state()
            rc = RCInputs(
                ch1_us=s.ch1_us,
                ch2_us=s.ch2_us,
                ch3_us=s.ch3_us,
                ch5_us=s.ch5_us,
                last_update_epoch_s=s.last_update_epoch_s,
            )
            # Prefer fresh BT over RC using timestamp freshness
            bt = bt_server.get_latest_bytes()
            bt_age = time.time() - bt.last_update_epoch_s
            bt_override = (bt.left_byte, bt.right_byte) if bt_age <= 0.6 else None
            cmd, events = controller.process(rc, bt_override_bytes=bt_override)
            # Emit debug lines on BT extrema updates
            if bt_override is not None:
                updated = False
                if bt.left_byte < min_btL:
                    min_btL = bt.left_byte
                    updated = True
                if bt.left_byte > max_btL:
                    max_btL = bt.left_byte
                    updated = True
                if bt.right_byte < min_btR:
                    min_btR = bt.right_byte
                    updated = True
                if bt.right_byte > max_btR:
                    max_btR = bt.right_byte
                    updated = True
                if updated:
                    print(
                        f"DBG BT extrema: L=[{min_btL},{max_btL}] R=[{min_btR},{max_btR}] (age={bt_age:0.2f}s)",
                        flush=True,
                    )
            # Minimal console heartbeat with incoming RC info
            src = "BT" if bt_override is not None else "RC"
            print(
                (
                    f"{src} ch1={s.ch1_us:4d} ch2={s.ch2_us:4d} ch3={s.ch3_us:4d} ch5={s.ch5_us:4d}  "
                    f"L={cmd.left_byte:3d} R={cmd.right_byte:3d} (bt_age={bt_age:0.2f}s btL={bt.left_byte:3d} btR={bt.right_byte:3d}) "
                    f"ARMED={cmd.is_armed} EMERG={cmd.emergency_active}    "
                ),
                end="\r",
                flush=True,
            )
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        print()
        try:
            bt_server.stop()
        except Exception:
            pass
        rc_reader.stop()


if __name__ == "__main__":
    run()


