import sys
import types


# Provide dummy hardware dependencies for tests
fake_ism = types.ModuleType("qwiic_ism330dhcx")
fake_ism.QwiicISM330DHCX = object
sys.modules.setdefault("qwiic_ism330dhcx", fake_ism)

fake_mmc = types.ModuleType("qwiic_mmc5983ma")
fake_mmc.QwiicMMC5983MA = object
sys.modules.setdefault("qwiic_mmc5983ma", fake_mmc)

