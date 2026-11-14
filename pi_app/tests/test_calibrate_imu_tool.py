"""The IMU calibration tool must read samples through the real ImuReader API
(mode/icm/ism/mmc), not the old ``reader.imu`` / ``reader.mag`` attributes."""
import types

from pi_app.cli import calibrate_imu as tool


class _FakeIsm:
    def get_gyro(self):
        return types.SimpleNamespace(xData=1000.0, yData=-2000.0, zData=500.0)

    def get_accel(self):
        return types.SimpleNamespace(xData=0.0, yData=0.0, zData=1000.0)


class _FakeReader:
    mode = "ISM_MMC"

    def __init__(self, *a, **k):
        self.ism = _FakeIsm()
        self.mmc = types.SimpleNamespace(
            get_measurement_xyz_gauss=lambda: (0.1, 0.2, 0.3))
        self.icm = None

    def _map_mag_axes(self, x, y, z):
        return x, y, -z


def _calibrator(monkeypatch, tmp_path):
    monkeypatch.setattr(tool, "ImuReader", _FakeReader)
    return tool.ImuCalibrator(str(tmp_path / "cal.json"))


def test_gyro_calibration_collects_samples(monkeypatch, tmp_path):
    c = _calibrator(monkeypatch, tmp_path)
    bias = c.calibrate_gyroscope(duration_s=0.2)
    assert bias == (1.0, -2.0, 0.5)
    assert c.imu.gyro_bias_dps == bias


def test_accel_and_mag_reads(monkeypatch, tmp_path):
    c = _calibrator(monkeypatch, tmp_path)
    assert c.read_accel_g() == (0.0, 0.0, 1.0)
    assert c.read_mag_gauss() == (0.1, 0.2, -0.3)


def test_uninitialised_reader_raises(monkeypatch, tmp_path):
    c = _calibrator(monkeypatch, tmp_path)
    c.imu.mode = None
    try:
        c.read_gyro_raw_dps()
    except RuntimeError:
        return
    raise AssertionError("expected RuntimeError")
