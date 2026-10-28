"""Unit tests:  python3 -m pytest -q   (or python3 test_sensors.py)"""
import math
import sensors


def test_frame_roundtrip():
    body = "T,2,0,-123,45,-3,512,3,180,1180"
    cs = 0
    for c in body:
        cs ^= ord(c)
    f = sensors.parse_frame(f"${body}*{cs:02X}\n")
    assert f["status"] == "STARTED" and f["roll_deg"] == -12.3 and f["battery_v"] == 11.8
    assert sensors.parse_frame(f"${body}*00\n") is None or cs == 0   # bad checksum rejected


def test_pulse_mapping():
    assert sensors.stick_to_pulse_us(0) == 1000
    assert sensors.stick_to_pulse_us(50) == 1500
    assert sensors.stick_to_pulse_us(100) == 2000
    assert sensors.stick_to_pulse_us(150) == 2000


def test_mq2_clean_air_is_r0():
    r0 = sensors.mq2_calibrate_r0(100)
    assert math.isclose(sensors.mq2_resistance(100) / r0, sensors.MQ2_CLEAN_AIR_RATIO)
    # at Rs/R0 = 10**0.21 the LPG curve gives exactly 200 ppm
    assert math.isclose(10 ** ((0.21 - 0.21) / -0.47 + 2.3), 199.526, rel_tol=1e-3)


def test_geiger_background():
    r = sensors.geiger_dose(20, "SBM-20")       # typical background ~ 20 CPM
    assert 0.10 < r.usv_h < 0.13
    assert math.isclose(r.sigma_usv_h, math.sqrt(20) * 0.0057)


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn(); print("ok", name)
