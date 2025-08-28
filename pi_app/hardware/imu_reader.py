#!/usr/bin/env python3


# Dependencies (install in your app env):
#   sparkfun-qwiic
#   sparkfun-qwiic-ism330dhcx
#   sparkfun-qwiic-mmc5983ma


import time, math, json
from pathlib import Path
from typing import Dict, Tuple, Optional
from qwiic_ism330dhcx import QwiicISM330DHCX
from qwiic_mmc5983ma import QwiicMMC5983MA


class ImuReader:
    def __init__(self, address_imu: int = 0x6B, address_mag: int = 0x30,
                 complementary_alpha_rp: float = 0.98,
                 complementary_alpha_yaw: float = 0.95,
                 calibration_path: Optional[str] = None) -> None:
        self.imu = QwiicISM330DHCX(address=address_imu)
        self.mag = QwiicMMC5983MA(address=address_mag)
        self.roll_rad = 0.0
        self.pitch_rad = 0.0
        self.yaw_rad = 0.0
        self.alpha_rp = float(complementary_alpha_rp)
        self.alpha_yaw = float(complementary_alpha_yaw)
        self.mag_offsets_g: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.gyro_bias_dps: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._last_monotonic: Optional[float] = None
        self.calibration_path = Path(calibration_path) if calibration_path else None
        if self.calibration_path and self.calibration_path.exists():
            try:
                self.load_calibration(self.calibration_path)
            except Exception:
                pass
        self._configure_devices()
        self._initialize_orientation_from_sensors()


    def read(self) -> Dict[str, float]:
        now = time.monotonic()
        if self._last_monotonic is None:
            self._last_monotonic = now
        dt = max(1e-3, now - self._last_monotonic)
        self._last_monotonic = now
        a = self.imu.get_accel()
        g = self.imu.get_gyro()
        mx_g, my_g, mz_g = self.mag.get_measurement_xyz_gauss()
        mz_g = -mz_g
        ax_g = (a.xData or 0.0)/1000.0
        ay_g = (a.yData or 0.0)/1000.0
        az_g = (a.zData or 0.0)/1000.0
        gx_dps = ((g.xData or 0.0)/1000.0) - self.gyro_bias_dps[0]
        gy_dps = ((g.yData or 0.0)/1000.0) - self.gyro_bias_dps[1]
        gz_dps = ((g.zData or 0.0)/1000.0) - self.gyro_bias_dps[2]
        mx_g -= self.mag_offsets_g[0]
        my_g -= self.mag_offsets_g[1]
        mz_g -= self.mag_offsets_g[2]
        roll_acc, pitch_acc, yaw_mag = self._tilt_compensated_heading(ax_g, ay_g, az_g, mx_g, my_g, mz_g)
        gx = math.radians(gx_dps)
        gy = math.radians(gy_dps)
        gz = math.radians(gz_dps)
        self.roll_rad = self.alpha_rp * (self.roll_rad + gx*dt) + (1.0-self.alpha_rp)*roll_acc
        self.pitch_rad = self.alpha_rp * (self.pitch_rad + gy*dt) + (1.0-self.alpha_rp)*pitch_acc
        yaw_pred = self.yaw_rad + gz*dt
        delta = (yaw_mag - yaw_pred + math.pi) % (2.0*math.pi) - math.pi
        self.yaw_rad = self.alpha_yaw * yaw_pred + (1.0-self.alpha_yaw)*(yaw_pred + delta)
        yaw_deg_ccw = math.degrees(self.yaw_rad)
        yaw_deg_cw = yaw_deg_ccw  # Fix: sensor readings are already in correct orientation
        heading_deg = (yaw_deg_cw + 360.0) % 360.0
        temp_c = self._read_temp_c()
        return {
            'roll_deg': math.degrees(self.roll_rad),
            'pitch_deg': math.degrees(self.pitch_rad),
            'yaw_deg': yaw_deg_cw,
            'heading_deg': heading_deg,
            'ax_g': ax_g, 'ay_g': ay_g, 'az_g': az_g,
            'gx_dps': gx_dps, 'gy_dps': gy_dps, 'gz_dps': gz_dps,
            'mx_g': mx_g, 'my_g': my_g, 'mz_g': mz_g,
            'temp_c': temp_c,
        }


    def calibrate_gyro(self, duration_s: float = 3.0):
        end_t = time.monotonic() + float(duration_s)
        xs, ys, zs = [], [], []
        while time.monotonic() < end_t:
            g = self.imu.get_gyro()
            xs.append((g.xData or 0.0)/1000.0)
            ys.append((g.yData or 0.0)/1000.0)
            zs.append((g.zData or 0.0)/1000.0)
            time.sleep(0.01)
        if xs:
            self.gyro_bias_dps = (sum(xs)/len(xs), sum(ys)/len(ys), sum(zs)/len(zs))
        return self.gyro_bias_dps


    def calibrate_mag_hard_iron(self, duration_s: float = 5.0):
        end_t = time.monotonic() + float(duration_s)
        minx = miny = minz = 1e9
        maxx = maxy = maxz = -1e9
        while time.monotonic() < end_t:
            mx, my, mz = self.mag.get_measurement_xyz_gauss()
            mz = -mz
            if mx < minx: minx = mx
            if my < miny: miny = my
            if mz < minz: minz = mz
            if mx > maxx: maxx = mx
            if my > maxy: maxy = my
            if mz > maxz: maxz = mz
            time.sleep(0.01)
        self.mag_offsets_g = ((maxx+minx)/2.0, (maxy+miny)/2.0, (maxz+minz)/2.0)
        return self.mag_offsets_g


    def save_calibration(self, path: Optional[Path] = None) -> None:
        target = Path(path) if path else (self.calibration_path or Path('imu_calibration.json'))
        data = {'mag_offsets_g': list(self.mag_offsets_g), 'gyro_bias_dps': list(self.gyro_bias_dps)}
        target.write_text(json.dumps(data, indent=2))


    def load_calibration(self, path: Optional[Path] = None) -> None:
        target = Path(path) if path else (self.calibration_path or Path('imu_calibration.json'))
        data = json.loads(target.read_text())
        self.mag_offsets_g = tuple(float(x) for x in data.get('mag_offsets_g', (0.0, 0.0, 0.0)))
        self.gyro_bias_dps = tuple(float(x) for x in data.get('gyro_bias_dps', (0.0, 0.0, 0.0)))


    def _configure_devices(self) -> None:
        self.imu.set_accel_full_scale(self.imu.kXlFs2g)
        self.imu.set_gyro_full_scale(self.imu.kGyroFs250dps)
        self.imu.set_accel_data_rate(self.imu.kXlOdr104Hz)
        self.imu.set_gyro_data_rate(self.imu.kGyroOdr104Hz)
        if not self.mag.begin():
            raise RuntimeError('Failed to initialize MMC5983MA magnetometer')


    def _initialize_orientation_from_sensors(self) -> None:
        a = self.imu.get_accel()
        ax_g, ay_g, az_g = (a.xData or 0.0)/1000.0, (a.yData or 0.0)/1000.0, (a.zData or 0.0)/1000.0
        mx_g, my_g, mz_g = self.mag.get_measurement_xyz_gauss()
        mz_g = -mz_g
        r, p, y = self._tilt_compensated_heading(ax_g, ay_g, az_g, mx_g - self.mag_offsets_g[0], my_g - self.mag_offsets_g[1], mz_g - self.mag_offsets_g[2])
        self.roll_rad, self.pitch_rad, self.yaw_rad = r, p, y
        self._last_monotonic = time.monotonic()


    @staticmethod
    def _tilt_compensated_heading(ax_g: float, ay_g: float, az_g: float, mx_g: float, my_g: float, mz_g: float):
        norm = math.sqrt(ax_g*ax_g + ay_g*ay_g + az_g*az_g) or 1.0
        ax = ax_g / norm; ay = ay_g / norm; az = az_g / norm
        roll = math.atan2(ay, az)
        pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az))
        mx2 = mx_g * math.cos(pitch) + mz_g * math.sin(pitch)
        my2 = (mx_g * math.sin(roll) * math.sin(pitch)) + (my_g * math.cos(roll)) - (mz_g * math.sin(roll) * math.cos(pitch))
        heading = math.atan2(-my2, mx2)
        return roll, pitch, heading


    def _read_temp_c(self) -> float:
        return self.imu.convert_lsb_to_celsius(self.imu.get_temp())