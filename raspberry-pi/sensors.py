"""
sensors.py — conversion of raw payload readings into physical quantities.

Pure functions (no hardware access) so they can be unit-tested on any PC.
See documentation/sensor-models.md for the derivations.

SPDX-License-Identifier: MIT
"""
from __future__ import annotations

import math
from dataclasses import dataclass

# ---------------------------------------------------------------------------
# MQ-2 metal-oxide gas sensor
# ---------------------------------------------------------------------------
# The sensing resistance Rs forms a divider with the load resistor RL:
#     Vout = Vc * RL / (Rs + RL)   =>   Rs = RL * (Vc - Vout) / Vout
# The datasheet sensitivity curves are straight lines in log-log space:
#     log10(Rs/R0) = m * (log10(ppm) - x0) + y0
#  => ppm = 10 ** ((log10(Rs/R0) - y0) / m + x0)
# Curve points (x0 = log10(200 ppm), y0 = log10(Rs/R0 at 200 ppm), slope m)
# digitised from the Hanwei MQ-2 datasheet (Fig. 2). Approximate (+/-30 %),
# valid 200-10 000 ppm, 20 degC / 65 %RH. NOT a calibrated instrument.
MQ2_CURVES = {
    "LPG":   (2.30, 0.21, -0.47),
    "CO":    (2.30, 0.72, -0.34),
    "smoke": (2.30, 0.53, -0.44),
}
MQ2_CLEAN_AIR_RATIO = 9.83   # Rs/R0 in clean air (datasheet)


def adc_to_volts(adc: int, vref: float = 5.0, bits: int = 10) -> float:
    """10-bit ATmega328P ADC: V = ADC * Vref / 1023."""
    return adc * vref / ((1 << bits) - 1)


def mq2_resistance(adc: int, rl_kohm: float = 1.0, vc: float = 5.0) -> float:
    """Sensor resistance Rs [kOhm] from the ADC reading (module RL is usually 1 kOhm)."""
    v = max(adc_to_volts(adc, vc), 1e-3)
    return rl_kohm * (vc - v) / v


def mq2_calibrate_r0(adc_clean_air: int, rl_kohm: float = 1.0) -> float:
    """R0 [kOhm] from a reading taken in clean air after >= 24 h burn-in / 3 min warm-up."""
    return mq2_resistance(adc_clean_air, rl_kohm) / MQ2_CLEAN_AIR_RATIO


def mq2_ppm(adc: int, r0_kohm: float, gas: str = "LPG", rl_kohm: float = 1.0) -> float:
    """Estimated concentration [ppm] of `gas` from the log-log datasheet curve."""
    x0, y0, m = MQ2_CURVES[gas]
    ratio = mq2_resistance(adc, rl_kohm) / r0_kohm
    if ratio <= 0:
        return float("nan")
    return 10 ** ((math.log10(ratio) - y0) / m + x0)


# ---------------------------------------------------------------------------
# Geiger–Müller counter
# ---------------------------------------------------------------------------
# Counting is a Poisson process: for N counts the standard deviation is sqrt(N).
# Dead-time (non-paralysable) correction:  n_true = n / (1 - n * tau)
# Dose-rate conversion (Cs-137 calibration, tube-specific):
#     H* [uSv/h] = CPM * CF
GM_TUBES = {
    #          CF [uSv/h per CPM]   dead time tau [s]
    "SBM-20": (0.0057,               190e-6),
    "J305":   (0.00812,              90e-6),
    "M4011":  (0.00650,              90e-6),
}


@dataclass
class RadiationReading:
    cpm: float          # dead-time-corrected counts per minute
    usv_h: float        # ambient dose-equivalent rate estimate [uSv/h]
    sigma_usv_h: float  # 1-sigma Poisson uncertainty [uSv/h]


def geiger_dose(counts_60s: int, tube: str = "SBM-20") -> RadiationReading:
    cf, tau = GM_TUBES[tube]
    n = counts_60s / 60.0                       # observed rate [counts/s]
    denom = 1.0 - n * tau
    n_true = n / denom if denom > 0.05 else float("inf")
    cpm = n_true * 60.0
    usv = cpm * cf
    sigma = math.sqrt(max(counts_60s, 1)) * cf  # sqrt(N) counts in a 60 s window
    return RadiationReading(cpm, usv, sigma)


# ---------------------------------------------------------------------------
# Telemetry frame from the flight controller
# $T,<status>,<fs>,<roll x10>,<pitch x10>,<yawrate x10>,<gas adc>,<cps>,<cpm>,<vbat cV>*<xor>
# ---------------------------------------------------------------------------
STATUS_NAMES = {0: "STOPPED", 1: "STARTING", 2: "STARTED"}


def parse_frame(line: str) -> dict | None:
    line = line.strip()
    if not line.startswith("$") or "*" not in line:
        return None
    body, cs_hex = line[1:].rsplit("*", 1)
    cs = 0
    for ch in body:
        cs ^= ord(ch)
    try:
        if cs != int(cs_hex, 16):
            return None
        f = body.split(",")
        if f[0] != "T" or len(f) != 10:
            return None
        v = [int(x) for x in f[1:]]
    except ValueError:
        return None
    return {
        "status": STATUS_NAMES.get(v[0], str(v[0])),
        "failsafe": bool(v[1]),
        "roll_deg": v[2] / 10.0,
        "pitch_deg": v[3] / 10.0,
        "yaw_rate_dps": v[4] / 10.0,
        "gas_adc": v[5],
        "geiger_cps": v[6],
        "geiger_counts_60s": v[7],
        "battery_v": v[8] / 100.0,
    }


def stick_to_pulse_us(value: float) -> int:
    """GUI stick value 0..100  ->  servo pulse 1000..2000 us (50 -> 1500 us centre)."""
    value = min(max(value, 0.0), 100.0)
    return int(round(1000 + 10 * value))
