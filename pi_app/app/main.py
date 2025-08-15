import time
import sys
from pathlib import Path

try:
    from pi_app.hardware.vesc import VescCanDriver
    from pi_app.hardware.arduino_modelx import ArduinoModelXDriver
    from pi_app.hardware.arduino_rc import ArduinoRCReader
    from pi_app.control.controller import Controller, RCInputs
except ModuleNotFoundError:
    # Allow running as a script: python3 /home/pi/pi_app/app/main.py
    sys.path.append(str(Path(__file__).resolve().parents[2]))
    from pi_app.hardware.vesc import VescCanDriver  # type: ignore
    from pi_app.hardware.arduino_modelx import ArduinoModelXDriver  # type: ignore
    from pi_app.hardware.arduino_rc import ArduinoRCReader  # type: ignore
    from pi_app.control.controller import Controller, RCInputs  # type: ignore


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
            cmd, events = controller.process(rc)
            # Minimal console heartbeat with incoming RC info
            print(
                (
                    f"ch1={s.ch1_us:4d} ch2={s.ch2_us:4d} ch3={s.ch3_us:4d} ch5={s.ch5_us:4d}  "
                    f"L={cmd.left_byte:3d} R={cmd.right_byte:3d} ARMED={cmd.is_armed} EMERG={cmd.emergency_active}    "
                ),
                end="\r",
                flush=True,
            )
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        print()
        rc_reader.stop()


if __name__ == "__main__":
    run()


