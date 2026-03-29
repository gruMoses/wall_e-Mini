#!/usr/bin/env python3

import logging
import os
import shutil
import subprocess
import time

import smbus2
from ina219 import INA219

# I2C configuration
DEVICE_BUS = 1

# Register map used by this UPS firmware family
REG_BAT_PROTECT_LOW = 17
REG_BAT_PROTECT_HIGH = 18
REG_SAMPLE_PERIOD_LOW = 21
REG_SAMPLE_PERIOD_HIGH = 22
REG_SHUTDOWN_COUNTDOWN = 24
REG_AUTO_POWER_ON = 25

# Retry / hardening behavior
I2C_RETRY_COUNT = 5
I2C_RETRY_DELAY_SECONDS = 0.15

# Behavior configuration
NO_CHARGE_GRACE_SECONDS = 30
UPS_SHUTDOWN_COUNTDOWN_SECONDS = 120
BOOT_GRACE_SECONDS = 10

# INA219 current threshold (mA) below which we consider AC absent.
# Battery reads negative when discharging; a small buffer avoids noise triggers.
AC_ABSENT_CURRENT_THRESHOLD_MA = -50.0

# 18650 Li-ion battery protection threshold (mV)
# 3200mV is a conservative cutoff that protects battery life and avoids deep discharge.
BATTERY_PROTECTION_MV = 3200


def detect_addr(bus: smbus2.SMBus) -> int:
    for addr in (0x17, 0x18):
        try:
            bus.read_byte_data(addr, 1)
            return addr
        except Exception:
            pass
    raise RuntimeError("UPS MCU not found at 0x17 or 0x18")


def read_reg_with_retry(bus: smbus2.SMBus, device_addr: int, register: int) -> int:
    for attempt in range(1, I2C_RETRY_COUNT + 1):
        try:
            return bus.read_byte_data(device_addr, register)
        except Exception as error:
            if attempt == I2C_RETRY_COUNT:
                raise RuntimeError(
                    f"read reg {register} failed after {I2C_RETRY_COUNT} tries: {error}"
                ) from error
            time.sleep(I2C_RETRY_DELAY_SECONDS)
    raise RuntimeError(f"unreachable read failure for register {register}")


def write_reg_verified(
    bus: smbus2.SMBus, device_addr: int, register: int, value: int
) -> bool:
    value &= 0xFF
    for attempt in range(1, I2C_RETRY_COUNT + 1):
        try:
            bus.write_byte_data(device_addr, register, value)
            readback = read_reg_with_retry(bus, device_addr, register)
            if readback == value:
                return True
            logging.warning(
                "I2C verify mismatch reg=%d wrote=%d read=%d attempt=%d/%d",
                register,
                value,
                readback,
                attempt,
                I2C_RETRY_COUNT,
            )
        except Exception as error:
            logging.warning(
                "I2C write/verify error reg=%d value=%d attempt=%d/%d err=%s",
                register,
                value,
                attempt,
                I2C_RETRY_COUNT,
                error,
            )
        time.sleep(I2C_RETRY_DELAY_SECONDS)
    return False


def detect_ac_present(
    bus: smbus2.SMBus, device_addr: int, ina_batt: INA219 | None = None
) -> bool:
    # Primary method: battery current direction via INA219.
    # Negative current means battery is discharging — AC is absent.
    if ina_batt is not None:
        try:
            current_ma = float(ina_batt.current())
            return current_ma >= AC_ABSENT_CURRENT_THRESHOLD_MA
        except Exception:
            pass
    # Fallback: UPS MCU charger port voltage registers.
    # Note: on some boards these read back the boost output (~5V) even
    # when no charger is connected, making them unreliable.
    receive_buffer = [0x00]
    for register_index in range(1, 11):
        receive_buffer.append(read_reg_with_retry(bus, device_addr, register_index))
    typec_mv = (receive_buffer[8] << 8) | receive_buffer[7]
    microusb_mv = (receive_buffer[10] << 8) | receive_buffer[9]
    return (typec_mv > 4000) or (microusb_mv > 4000)


def read_ups_snapshot(
    bus: smbus2.SMBus, device_addr: int, ina_batt: INA219 | None = None
) -> dict[str, float | int | None]:
    regs = [read_reg_with_retry(bus, device_addr, idx) for idx in range(1, 11)]
    typec_mv = (regs[7] << 8) | regs[6]
    microusb_mv = (regs[9] << 8) | regs[8]
    protect_low = read_reg_with_retry(bus, device_addr, REG_BAT_PROTECT_LOW)
    protect_high = read_reg_with_retry(bus, device_addr, REG_BAT_PROTECT_HIGH)
    protect_mv = (protect_high << 8) | protect_low
    countdown = read_reg_with_retry(bus, device_addr, REG_SHUTDOWN_COUNTDOWN)
    auto_on = read_reg_with_retry(bus, device_addr, REG_AUTO_POWER_ON)
    sample_low = read_reg_with_retry(bus, device_addr, REG_SAMPLE_PERIOD_LOW)
    sample_high = read_reg_with_retry(bus, device_addr, REG_SAMPLE_PERIOD_HIGH)
    sample_minutes = (sample_high << 8) | sample_low

    batt_v = None
    batt_i = None
    if ina_batt is not None:
        try:
            batt_v = round(float(ina_batt.voltage()), 3)
            batt_i = round(float(ina_batt.current()), 1)
        except Exception:
            pass

    return {
        "typec_mv": typec_mv,
        "microusb_mv": microusb_mv,
        "protect_mv": protect_mv,
        "shutdown_countdown_s": countdown,
        "auto_power_on": auto_on,
        "sample_period_min": sample_minutes,
        "battery_v": batt_v,
        "battery_i_ma": batt_i,
    }


def stop_rover_service() -> None:
    """Stop the wall-e service so OAK-D pipeline shuts down cleanly."""
    logging.info("Stopping wall-e.service for clean OAK-D teardown...")
    try:
        result = subprocess.run(
            ["sudo", "systemctl", "stop", "wall-e.service"],
            timeout=15,
            capture_output=True,
            text=True,
        )
        if result.returncode == 0:
            logging.info("wall-e.service stopped.")
        else:
            logging.warning(
                "wall-e.service stop returned %d: %s",
                result.returncode,
                result.stderr.strip(),
            )
    except subprocess.TimeoutExpired:
        logging.warning("wall-e.service stop timed out after 15s.")
    except Exception as error:
        logging.warning("Failed to stop wall-e.service: %s", error)


def shed_usb_load() -> None:
    """Cut USB power to the OAK-D hub to reduce battery drain.

    Pi 5 root hubs support per-port power switching.  The OAK-D sits
    behind a Genesys hub on Bus 3 Port 1 (USB 2.0) / Bus 4 Port 1
    (USB 3.0 companion).  The VESC CH340 is on Bus 3 Port 2 and is
    left untouched.
    """
    uhubctl = shutil.which("uhubctl")
    if uhubctl:
        for loc, port, label in [
            ("3", "1", "USB2.0 hub (OAK-D)"),
            ("4", "1", "USB3.0 companion hub"),
        ]:
            try:
                result = subprocess.run(
                    ["sudo", uhubctl, "-l", loc, "-p", port, "-a", "off"],
                    timeout=5,
                    capture_output=True,
                    text=True,
                )
                if result.returncode == 0:
                    logging.info("USB power off: %s (location %s port %s).", label, loc, port)
                else:
                    logging.warning(
                        "uhubctl power off failed for %s: %s", label, result.stderr.strip()
                    )
            except Exception as error:
                logging.warning("uhubctl error for %s: %s", label, error)
    else:
        logging.info("uhubctl not found; falling back to sysfs deauthorize.")
        for devpath, label in [
            ("/sys/bus/usb/devices/3-1.3/authorized", "OAK-D (3-1.3)"),
            ("/sys/bus/usb/devices/3-1/authorized", "USB2.0 hub (3-1)"),
            ("/sys/bus/usb/devices/4-1/authorized", "USB3.0 hub (4-1)"),
        ]:
            try:
                with open(devpath, "w") as f:
                    f.write("0")
                logging.info("Deauthorized %s via sysfs.", label)
            except Exception as error:
                logging.warning("sysfs deauthorize failed for %s: %s", label, error)


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s: %(message)s")
    bus = smbus2.SMBus(DEVICE_BUS)
    device_addr = detect_addr(bus)
    ina_batt = INA219(0.005, busnum=DEVICE_BUS, address=0x45)
    ina_batt.configure()

    # Log initial observed config for traceability.
    try:
        snap = read_ups_snapshot(bus, device_addr, ina_batt)
        logging.info(
            "UPS initial snapshot: typec=%smV microusb=%smV protect=%smV auto_on=%s "
            "countdown=%ss sample=%smin batt=%sV %smA",
            snap["typec_mv"],
            snap["microusb_mv"],
            snap["protect_mv"],
            snap["auto_power_on"],
            snap["shutdown_countdown_s"],
            snap["sample_period_min"],
            snap["battery_v"],
            snap["battery_i_ma"],
        )
    except Exception as error:
        logging.warning("Unable to read UPS initial snapshot: %s", error)

    # Ensure auto power-on when AC returns.
    if write_reg_verified(bus, device_addr, REG_AUTO_POWER_ON, 1):
        logging.info("Back-to-AC auto power-on enabled.")
    else:
        logging.warning("Failed to enable Back-to-AC after retries.")

    # Ensure battery protection threshold is set and verified.
    protect_low = BATTERY_PROTECTION_MV & 0xFF
    protect_high = (BATTERY_PROTECTION_MV >> 8) & 0xFF
    low_ok = write_reg_verified(bus, device_addr, REG_BAT_PROTECT_LOW, protect_low)
    high_ok = write_reg_verified(bus, device_addr, REG_BAT_PROTECT_HIGH, protect_high)
    if low_ok and high_ok:
        try:
            low_read = read_reg_with_retry(bus, device_addr, REG_BAT_PROTECT_LOW)
            high_read = read_reg_with_retry(bus, device_addr, REG_BAT_PROTECT_HIGH)
            protect_readback = (high_read << 8) | low_read
            logging.info("Battery protection threshold set/readback: %smV", protect_readback)
        except Exception as error:
            logging.warning("Battery protection threshold set but readback failed: %s", error)
    else:
        logging.warning(
            "Failed to set battery protection threshold to %smV after retries.",
            BATTERY_PROTECTION_MV,
        )

    seconds_without_charge = 0
    boot_elapsed = 0
    last_ac_present = None

    while True:
        try:
            ac_present = detect_ac_present(bus, device_addr, ina_batt)
        except Exception as error:
            logging.warning("I2C read error: %s", error)
            # Small delay to avoid busy-loop on persistent errors.
            time.sleep(2)
            continue

        if boot_elapsed < BOOT_GRACE_SECONDS:
            boot_elapsed += 1
            time.sleep(1)
            continue

        if last_ac_present is None:
            last_ac_present = ac_present
            logging.info("UPS AC state initialized: %s", "present" if ac_present else "missing")
        elif ac_present != last_ac_present:
            last_ac_present = ac_present
            try:
                snap = read_ups_snapshot(bus, device_addr, ina_batt)
                logging.info(
                    "UPS AC state changed -> %s (typec=%smV microusb=%smV batt=%sV %smA)",
                    "present" if ac_present else "missing",
                    snap["typec_mv"],
                    snap["microusb_mv"],
                    snap["battery_v"],
                    snap["battery_i_ma"],
                )
            except Exception as error:
                logging.info("UPS AC state changed -> %s (snapshot failed: %s)", ac_present, error)

        if ac_present:
            if seconds_without_charge != 0:
                logging.info("Charger present again; resetting grace timer.")
            seconds_without_charge = 0
        else:
            seconds_without_charge += 1
            if seconds_without_charge == 1:
                logging.info(
                    "Charger appears absent; starting %ss glitch grace window.",
                    NO_CHARGE_GRACE_SECONDS,
                )
            if seconds_without_charge == NO_CHARGE_GRACE_SECONDS:
                try:
                    snap = read_ups_snapshot(bus, device_addr, ina_batt)
                    logging.warning(
                        "No charger for %ss; safe shutdown sequence begins. "
                        "typec=%smV microusb=%smV protect=%smV batt=%sV %smA",
                        NO_CHARGE_GRACE_SECONDS,
                        snap["typec_mv"],
                        snap["microusb_mv"],
                        snap["protect_mv"],
                        snap["battery_v"],
                        snap["battery_i_ma"],
                    )
                except Exception as error:
                    logging.warning(
                        "No charger for %ss; safe shutdown sequence begins (snapshot failed: %s).",
                        NO_CHARGE_GRACE_SECONDS,
                        error,
                    )

                stop_rover_service()
                shed_usb_load()

                logging.info(
                    "Setting UPS shutdown countdown to %ss.",
                    UPS_SHUTDOWN_COUNTDOWN_SECONDS,
                )
                # Request UPS to cut power after countdown; with Back-to-AC enabled it
                # powers back on only when AC returns.
                if write_reg_verified(
                    bus,
                    device_addr,
                    REG_SHUTDOWN_COUNTDOWN,
                    UPS_SHUTDOWN_COUNTDOWN_SECONDS,
                ):
                    logging.info(
                        "UPS shutdown countdown verified at %ss.",
                        UPS_SHUTDOWN_COUNTDOWN_SECONDS,
                    )
                else:
                    logging.warning(
                        "Failed to set UPS shutdown countdown after retries; proceeding with OS shutdown."
                    )

                logging.info("Running sync and OS halt now.")
                os.system("sudo sync")
                os.system("sudo /sbin/shutdown -h now")
                # Give systemd time; if still running, wait to avoid repeated triggers.
                time.sleep(600)
                # After halt, we should not reach here under normal conditions.

        time.sleep(1)


if __name__ == "__main__":
    main()