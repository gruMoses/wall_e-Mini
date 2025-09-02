#!/usr/bin/env python3


# Dependencies (install in your app env):
#   sparkfun-qwiic
#   sparkfun-qwiic-icm20948


import time, math, json
from pathlib import Path
from typing import Dict, Tuple, Optional
from importlib import import_module


class ImuReader:
    def __init__(self, address_imu: int = 0x69,
                 complementary_alpha_rp: float = 0.98,
                 complementary_alpha_yaw: float = 0.95,
                 calibration_path: Optional[str] = None,
                 mag_axis_map: Optional[Tuple[str, str, str]] = None,
                 heading_cw_positive: bool = True) -> None:
        # Hardware can be either single-chip ICM-20948 or ISM330DHCX+MMC5983MA combo.
        # We'll auto-detect and initialize accordingly.
        self.mode: Optional[str] = None  # 'ICM20948' or 'ISM_MMC'
        self.icm = None
        self.ism = None
        self.mmc = None
        # Track address preference (for ICM-20948)
        self._pref_addr_icm = address_imu
        self.roll_rad = 0.0
        self.pitch_rad = 0.0
        self.yaw_rad = 0.0
        self.alpha_rp = float(complementary_alpha_rp)
        self.alpha_yaw = float(complementary_alpha_yaw)
        self.mag_offsets_g: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.mag_scales: Tuple[float, float, float] = (1.0, 1.0, 1.0)
        self.gyro_bias_dps: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._last_monotonic: Optional[float] = None
        # Resolve calibration path in a stable way regardless of CWD:
        # - If an absolute path is provided, use it.
        # - If a relative path (or None) is provided, resolve it relative to the
        #   project root (two levels above this file: wall_e-Mini/...).
        project_root = Path(__file__).resolve().parents[2]
        if calibration_path:
            p = Path(calibration_path)
            self.calibration_path = p if p.is_absolute() else (project_root / p)
        else:
            self.calibration_path = project_root / 'imu_calibration.json'
        self.mag_axis_map: Optional[Tuple[str, str, str]] = mag_axis_map
        self._heading_cw_positive = bool(heading_cw_positive)
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

        if self.mode == 'ICM20948':
            imu = self.icm
            if getattr(imu, 'dataReady', lambda: False)():
                imu.getAgmt()
            # Scale raw readings to physical units based on default ranges set in begin():
            #  - Accel ±2g => 16384 LSB/g
            #  - Gyro ±250 dps => 131 LSB/(deg/s)
            #  - Mag (AK09916) ~0.15 µT/LSB => 0.0015 Gauss/LSB
            ax_g = float(getattr(imu, 'axRaw', 0.0)) / 16384.0
            ay_g = float(getattr(imu, 'ayRaw', 0.0)) / 16384.0
            az_g = float(getattr(imu, 'azRaw', 0.0)) / 16384.0

            gx_dps = (float(getattr(imu, 'gxRaw', 0.0)) / 131.0) - self.gyro_bias_dps[0]
            gy_dps = (float(getattr(imu, 'gyRaw', 0.0)) / 131.0) - self.gyro_bias_dps[1]
            gz_dps = (float(getattr(imu, 'gzRaw', 0.0)) / 131.0) - self.gyro_bias_dps[2]

            mx = float(getattr(imu, 'mxRaw', 0.0)) * 0.0015
            my = float(getattr(imu, 'myRaw', 0.0)) * 0.0015
            mz = float(getattr(imu, 'mzRaw', 0.0)) * 0.0015
            mx, my, mz = self._map_mag_axes(mx, my, mz)
            mx_g = (mx - self.mag_offsets_g[0]) * self.mag_scales[0]
            my_g = (my - self.mag_offsets_g[1]) * self.mag_scales[1]
            mz_g = (mz - self.mag_offsets_g[2]) * self.mag_scales[2]
        elif self.mode == 'ISM_MMC':
            # ISM330DHCX accel/gyro and MMC5983MA magnetometer
            a = self.ism.get_accel()
            g = self.ism.get_gyro()
            mx, my, mz = self.mmc.get_measurement_xyz_gauss()
            # Map axes (defaults to invert Z for ISM+MMC)
            mx, my, mz = self._map_mag_axes(mx, my, mz)
            # ISM library returns milli-units; convert to g and dps
            ax_g = (getattr(a, 'xData', 0.0) or 0.0) / 1000.0
            ay_g = (getattr(a, 'yData', 0.0) or 0.0) / 1000.0
            az_g = (getattr(a, 'zData', 0.0) or 0.0) / 1000.0
            gx_dps = ((getattr(g, 'xData', 0.0) or 0.0) / 1000.0) - self.gyro_bias_dps[0]
            gy_dps = ((getattr(g, 'yData', 0.0) or 0.0) / 1000.0) - self.gyro_bias_dps[1]
            gz_dps = ((getattr(g, 'zData', 0.0) or 0.0) / 1000.0) - self.gyro_bias_dps[2]
            # Apply mag hard-iron offsets and soft-iron scaling
            mx_g = (mx - self.mag_offsets_g[0]) * self.mag_scales[0]
            my_g = (my - self.mag_offsets_g[1]) * self.mag_scales[1]
            mz_g = (mz - self.mag_offsets_g[2]) * self.mag_scales[2]
        else:
            raise RuntimeError('IMU not initialized')

        # Compute orientation via complementary filter
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
        yaw_display = (-yaw_deg_ccw) if self._heading_cw_positive else yaw_deg_ccw
        heading_deg = (yaw_display + 360.0) % 360.0
        temp_c = self._read_temp_c()
        return {
            'roll_deg': math.degrees(self.roll_rad),
            'pitch_deg': math.degrees(self.pitch_rad),
            'yaw_deg': yaw_display,
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
            if self.mode == 'ICM20948':
                imu = self.icm
                if getattr(imu, 'dataReady', lambda: False)():
                    imu.getAgmt()
                xs.append(float(getattr(imu, 'gxRaw', 0.0)) / 131.0)
                ys.append(float(getattr(imu, 'gyRaw', 0.0)) / 131.0)
                zs.append(float(getattr(imu, 'gzRaw', 0.0)) / 131.0)
            else:
                g = self.ism.get_gyro()
                xs.append((getattr(g, 'xData', 0.0) or 0.0) / 1000.0)
                ys.append((getattr(g, 'yData', 0.0) or 0.0) / 1000.0)
                zs.append((getattr(g, 'zData', 0.0) or 0.0) / 1000.0)
            time.sleep(0.01)
        if xs:
            self.gyro_bias_dps = (sum(xs)/len(xs), sum(ys)/len(ys), sum(zs)/len(zs))
        return self.gyro_bias_dps


    def calibrate_mag_hard_iron(self, duration_s: float = 5.0):
        end_t = time.monotonic() + float(duration_s)
        minx = miny = minz = 1e9
        maxx = maxy = maxz = -1e9
        while time.monotonic() < end_t:
            if self.mode == 'ICM20948':
                imu = self.icm
                if getattr(imu, 'dataReady', lambda: False)():
                    imu.getAgmt()
                mx = float(getattr(imu, 'mxRaw', 0.0)) * 0.0015
                my = float(getattr(imu, 'myRaw', 0.0)) * 0.0015
                mz = float(getattr(imu, 'mzRaw', 0.0)) * 0.0015
            else:
                mx, my, mz = self.mmc.get_measurement_xyz_gauss()
            mx, my, mz = self._map_mag_axes(mx, my, mz)
            if mx < minx: minx = mx
            if my < miny: miny = my
            if mz < minz: minz = mz
            if mx > maxx: maxx = mx
            if my > maxy: maxy = my
            if mz > maxz: maxz = mz
            time.sleep(0.01)
        self.mag_offsets_g = ((maxx+minx)/2.0, (maxy+miny)/2.0, (maxz+minz)/2.0)
        return self.mag_offsets_g


    def calibrate_mag(self, duration_s: float = 8.0):
        """Compute both hard‑iron offsets and soft‑iron scale factors.
        Move through full 3D orientations during collection.
        """
        end_t = time.monotonic() + float(duration_s)
        minx = miny = minz = 1e9
        maxx = maxy = maxz = -1e9
        while time.monotonic() < end_t:
            if self.mode == 'ICM20948':
                imu = self.icm
                if getattr(imu, 'dataReady', lambda: False)():
                    imu.getAgmt()
                mx = float(getattr(imu, 'mxRaw', 0.0)) * 0.0015
                my = float(getattr(imu, 'myRaw', 0.0)) * 0.0015
                mz = float(getattr(imu, 'mzRaw', 0.0)) * 0.0015
            else:
                mx, my, mz = self.mmc.get_measurement_xyz_gauss()
            mx, my, mz = self._map_mag_axes(mx, my, mz)
            if mx < minx: minx = mx
            if my < miny: miny = my
            if mz < minz: minz = mz
            if mx > maxx: maxx = mx
            if my > maxy: maxy = my
            if mz > maxz: maxz = mz
            time.sleep(0.01)
        # Hard‑iron offsets
        offx = (maxx + minx) / 2.0
        offy = (maxy + miny) / 2.0
        offz = (maxz + minz) / 2.0
        self.mag_offsets_g = (offx, offy, offz)
        # Soft‑iron scales by normalizing radii
        sx = (maxx - minx) / 2.0
        sy = (maxy - miny) / 2.0
        sz = (maxz - minz) / 2.0
        avg = max(1e-6, (sx + sy + sz) / 3.0)
        self.mag_scales = (
            avg / max(1e-6, sx),
            avg / max(1e-6, sy),
            avg / max(1e-6, sz),
        )
        return self.mag_offsets_g, self.mag_scales


    def save_calibration(self, path: Optional[Path] = None) -> None:
        target = Path(path) if path else self.calibration_path
        data = {
            'mag_offsets_g': list(self.mag_offsets_g),
            'mag_scales': list(self.mag_scales),
            'gyro_bias_dps': list(self.gyro_bias_dps),
        }
        target.write_text(json.dumps(data, indent=2))


    def load_calibration(self, path: Optional[Path] = None) -> None:
        target = Path(path) if path else self.calibration_path
        data = json.loads(target.read_text())
        self.mag_offsets_g = tuple(float(x) for x in data.get('mag_offsets_g', (0.0, 0.0, 0.0)))
        self.mag_scales = tuple(float(x) for x in data.get('mag_scales', (1.0, 1.0, 1.0)))
        self.gyro_bias_dps = tuple(float(x) for x in data.get('gyro_bias_dps', (0.0, 0.0, 0.0)))


    def _configure_devices(self) -> None:
        # Prefer ICM-20948 if present; otherwise use ISM330DHCX + MMC5983MA combo.
        # We import drivers lazily to avoid hard dependency at import time.
        tried_icm = []
        try:
            qwiic_icm20948 = import_module('qwiic_icm20948')
        except Exception:
            qwiic_icm20948 = None

        # Try ICM addresses if driver is available
        if qwiic_icm20948 is not None:
            def try_init_icm(addr: int) -> bool:
                dev = qwiic_icm20948.QwiicIcm20948(address=addr)
                tried_icm.append(f"0x{addr:02X}")
                if not getattr(dev, 'connected', False):
                    return False
                if not bool(dev.begin()):
                    return False
                self.icm = dev
                self.mode = 'ICM20948'
                return True

            # Try preferred then alternate
            current_addr = self._pref_addr_icm
            if not try_init_icm(current_addr):
                alt = 0x68 if current_addr == 0x69 else 0x69
                try_init_icm(alt)

        if self.mode != 'ICM20948':
            # Fall back to ISM330DHCX + MMC5983MA
            try:
                QwiicISM330DHCX = import_module('qwiic_ism330dhcx').QwiicISM330DHCX
                QwiicMMC5983MA = import_module('qwiic_mmc5983ma').QwiicMMC5983MA
            except Exception as e:
                # If we already tried ICM and failed, surface helpful message
                if tried_icm:
                    raise RuntimeError(
                        "IMU drivers not available: tried ICM-20948 (addresses: " + 
                        ', '.join(tried_icm) + ") and ISM330DHCX/MMC5983MA, but drivers missing."
                    ) from e
                raise

            # Initialize ISM330DHCX
            self.ism = QwiicISM330DHCX(address=0x6B)
            # Configure ranges and data rates to reasonable defaults
            self.ism.set_accel_full_scale(self.ism.kXlFs2g)
            self.ism.set_gyro_full_scale(self.ism.kGyroFs250dps)
            self.ism.set_accel_data_rate(self.ism.kXlOdr104Hz)
            self.ism.set_gyro_data_rate(self.ism.kGyroOdr104Hz)

            # Initialize MMC5983MA magnetometer
            self.mmc = QwiicMMC5983MA(address=0x30)
            if not self.mmc.begin():
                raise RuntimeError('Failed to initialize MMC5983MA magnetometer (addr 0x30)')
            self.mode = 'ISM_MMC'


    def _initialize_orientation_from_sensors(self) -> None:
        if self.mode == 'ICM20948':
            imu = self.icm
            if getattr(imu, 'dataReady', lambda: False)():
                imu.getAgmt()
            ax_g = float(getattr(imu, 'axRaw', 0.0)) / 16384.0
            ay_g = float(getattr(imu, 'ayRaw', 0.0)) / 16384.0
            az_g = float(getattr(imu, 'azRaw', 0.0)) / 16384.0
            mx = float(getattr(imu, 'mxRaw', 0.0)) * 0.0015
            my = float(getattr(imu, 'myRaw', 0.0)) * 0.0015
            mz = float(getattr(imu, 'mzRaw', 0.0)) * 0.0015
            mx, my, mz = self._map_mag_axes(mx, my, mz)
            mx_g = (mx - self.mag_offsets_g[0]) * self.mag_scales[0]
            my_g = (my - self.mag_offsets_g[1]) * self.mag_scales[1]
            mz_g = (mz - self.mag_offsets_g[2]) * self.mag_scales[2]
        elif self.mode == 'ISM_MMC':
            a = self.ism.get_accel()
            ax_g = (getattr(a, 'xData', 0.0) or 0.0) / 1000.0
            ay_g = (getattr(a, 'yData', 0.0) or 0.0) / 1000.0
            az_g = (getattr(a, 'zData', 0.0) or 0.0) / 1000.0
            mx, my, mz = self.mmc.get_measurement_xyz_gauss()
            mx, my, mz = self._map_mag_axes(mx, my, mz)
            mx_g = (mx - self.mag_offsets_g[0]) * self.mag_scales[0]
            my_g = (my - self.mag_offsets_g[1]) * self.mag_scales[1]
            mz_g = (mz - self.mag_offsets_g[2]) * self.mag_scales[2]
        else:
            raise RuntimeError('IMU not initialized')
        r, p, y = self._tilt_compensated_heading(ax_g, ay_g, az_g, mx_g, my_g, mz_g)
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
        if self.mode == 'ICM20948':
            # ICM-20948 temperature sensitivity is ~333.87 LSB/°C with a 21°C offset reference.
            # Approximate conversion to degrees Celsius from raw counts.
            tmp_raw = float(getattr(self.icm, 'tmpRaw', 0.0))
            return ((tmp_raw - 21.0) / 333.87) + 21.0
        else:
            # Use ISM330DHCX helper
            try:
                return self.ism.convert_lsb_to_celsius(self.ism.get_temp())
            except Exception:
                return 0.0

    # --- Axis mapping helpers ---
    def _map_mag_axes(self, mx: float, my: float, mz: float) -> Tuple[float, float, float]:
        """Map raw magnetometer axes into the accel/gyro coordinate frame.
        mag_axis_map is a tuple of 3 strings: 'x','-x','y','-y','z','-z'.
        Defaults: ISM+MMC ('x','y','-z'), ICM ('x','y','z').
        """
        m = self._get_effective_mag_axis_map()
        def sel(tag: str) -> float:
            s = -1.0 if tag.startswith('-') else 1.0
            axis = tag[1:] if tag.startswith('-') else tag
            if axis == 'x':
                return s * mx
            if axis == 'y':
                return s * my
            if axis == 'z':
                return s * mz
            return 0.0
        return (sel(m[0]), sel(m[1]), sel(m[2]))

    def _get_effective_mag_axis_map(self) -> Tuple[str, str, str]:
        if self.mag_axis_map:
            return self.mag_axis_map
        if self.mode == 'ISM_MMC':
            return ('x', 'y', '-z')
        return ('x', 'y', 'z')
