"""The OAK session opens on a device capped at the configured USB speed.

2026-11-06: the USB 3 link jammed the RTK GPS, so wall-e caps the OAK at
USB 2 (OakDetectionConfig.max_usb_speed = "HIGH"). These tests use a stand-in
depthai module; they check which constructor runs, not the hardware.
"""

from types import SimpleNamespace

from config import OakDetectionConfig
from pi_app.hardware import oak_depth
from pi_app.hardware.oak_depth import open_pipeline


class _FakeDai:
    def __init__(self, with_usb_speed=True):
        self.calls = []
        if with_usb_speed:
            self.UsbSpeed = SimpleNamespace(HIGH="usb-high", SUPER="usb-super")

        dai = self

        class Device:
            def __init__(self, *args):
                dai.calls.append(("Device", args))

        class Pipeline:
            def __init__(self, *args):
                dai.calls.append(("Pipeline", args))
                self.args = args

        self.Device = Device
        self.Pipeline = Pipeline


def test_config_default_is_usb2():
    assert OakDetectionConfig().max_usb_speed == "HIGH"
    assert OakDetectionConfig.max_usb_speed == "HIGH"


def test_high_opens_a_device_capped_at_usb2():
    dai = _FakeDai()
    pipeline = open_pipeline(dai, "HIGH")
    assert dai.calls[0] == ("Device", ("usb-high",))
    assert dai.calls[1][0] == "Pipeline"
    assert len(pipeline.args) == 1  # the capped device, not an implicit one


def test_name_is_case_and_space_insensitive():
    dai = _FakeDai()
    open_pipeline(dai, " super ")
    assert dai.calls[0] == ("Device", ("usb-super",))


def test_empty_keeps_the_depthai_default():
    dai = _FakeDai()
    pipeline = open_pipeline(dai, "")
    assert dai.calls == [("Pipeline", ())]
    assert pipeline.args == ()


def test_unknown_name_keeps_the_default_and_warns(caplog):
    dai = _FakeDai()
    with caplog.at_level("WARNING"):
        open_pipeline(dai, "USB3")
    assert dai.calls == [("Pipeline", ())]
    assert "not a depthai UsbSpeed" in caplog.text


def test_depthai_without_usbspeed_keeps_the_default():
    dai = _FakeDai(with_usb_speed=False)
    open_pipeline(dai, "HIGH")
    assert dai.calls == [("Pipeline", ())]


def test_reader_cap_follows_config_and_falls_back_to_default():
    reader = oak_depth.OakDepthReader.__new__(oak_depth.OakDepthReader)
    reader._det_cfg = None
    assert reader._max_usb_speed() == "HIGH"
    reader._det_cfg = SimpleNamespace(max_usb_speed="super")
    assert reader._max_usb_speed() == "SUPER"
    reader._det_cfg = SimpleNamespace(max_usb_speed="")
    assert reader._max_usb_speed() == ""
