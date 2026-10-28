#!/usr/bin/env python3
"""
bridge.py — on-board companion computer (Raspberry Pi 4) software.

  Firebase  /drones/<id>/command   --(poll 10 Hz)-->  servo pulses 1000..2000 us  --> ATmega328P
  ATmega328P UART telemetry frames --(parse, convert)--> /drones/<id>/telemetry/latest (+ history 1 Hz)

Improvements over the 2022 script
  * pulses generated with pigpio (DMA-timed, jitter < 5 us) instead of RPi.GPIO
    software PWM (jitter of tens of us, visible as stick noise);
  * named JSON fields instead of parsing str(dict) character by character
    (the 2022 parser read the fields in a different order than the Java app
    wrote them: pitch|roll|yaw|throttle was interpreted as yaw|roll|pitch|throttle);
  * authenticated access (Firebase Auth) instead of an open database;
  * link watchdog: if no fresh command arrives for `command_timeout` seconds the
    pulses are switched off, which triggers the flight controller fail-safe
    (level attitude, throttle ramp-down, disarm);
  * telemetry upload (gas, radiation, attitude, battery) — the thesis described
    it but the 2022 code never sent measurements to the database.

Run:   python3 bridge.py --config config.ini
       python3 bridge.py --config config.ini --simulate     (no Pi hardware needed)

SPDX-License-Identifier: MIT
"""
from __future__ import annotations

import argparse
import configparser
import logging
import math
import random
import signal
import sys
import threading
import time

import sensors
from firebase_client import FirebaseRTDB

log = logging.getLogger("bridge")

# BCM numbers of the physical header pins used in the thesis (GPIO.BOARD 12/13/18/19)
DEFAULT_PINS = {"yaw": 18, "roll": 27, "pitch": 24, "throttle": 10}
NEUTRAL = {"roll": 50, "pitch": 50, "yaw": 50, "throttle": 0}


# ---------------------------------------------------------------------------
# Pulse outputs
# ---------------------------------------------------------------------------
class PulseOutput:
    """Servo-style pulses on 4 GPIOs (50 Hz, 1000..2000 us)."""

    def __init__(self, pins: dict, simulate: bool):
        self.pins, self.simulate = pins, simulate
        self.pi = None
        if not simulate:
            import pigpio  # sudo apt install pigpio python3-pigpio ; sudo systemctl enable --now pigpiod
            self.pi = pigpio.pi()
            if not self.pi.connected:
                raise RuntimeError("pigpiod is not running (sudo systemctl start pigpiod)")
        self.last = {}

    def write(self, sticks: dict) -> None:
        for axis, pin in self.pins.items():
            us = sensors.stick_to_pulse_us(sticks[axis])
            self.last[axis] = us
            if self.pi:
                self.pi.set_servo_pulsewidth(pin, us)

    def off(self) -> None:
        """No pulses at all -> the flight controller detects link loss."""
        self.last = {}
        if self.pi:
            for pin in self.pins.values():
                self.pi.set_servo_pulsewidth(pin, 0)

    def close(self) -> None:
        self.off()
        if self.pi:
            self.pi.stop()


# ---------------------------------------------------------------------------
# Telemetry input
# ---------------------------------------------------------------------------
class TelemetryReader(threading.Thread):
    """Reads $T frames from the ATmega328P UART (or fabricates them in --simulate)."""

    def __init__(self, port: str, baud: int, simulate: bool, out_cb=None):
        super().__init__(daemon=True)
        self.port, self.baud, self.simulate = port, baud, simulate
        self.latest: dict | None = None
        self.frames = self.bad = 0
        self.out = None
        self._stop = threading.Event()
        self.sim_sticks = dict(NEUTRAL)

    def run(self) -> None:
        if self.simulate:
            return self._run_sim()
        import serial  # pip install pyserial
        with serial.Serial(self.port, self.baud, timeout=1) as ser:
            while not self._stop.is_set():
                line = ser.readline().decode("ascii", errors="replace")
                if not line:
                    continue
                frame = sensors.parse_frame(line)
                if frame:
                    frame["rx_time"] = time.time()
                    self.latest = frame
                    self.frames += 1
                else:
                    self.bad += 1

    def _run_sim(self) -> None:
        t0 = time.time()
        counts = [random.randint(15, 30) for _ in range(60)]
        while not self._stop.is_set():
            t = time.time() - t0
            counts = counts[1:] + [sum(1 for _ in range(60) if random.random() < 0.0055 * 60 / 60)]
            s = self.sim_sticks
            self.latest = {
                "status": "STARTED" if s["throttle"] > 0 else "STOPPED",
                "failsafe": False,
                "roll_deg": (s["roll"] - 50) * 0.66 + 0.4 * math.sin(t * 3.1),
                "pitch_deg": (s["pitch"] - 50) * 0.66 + 0.4 * math.cos(t * 2.7),
                "yaw_rate_dps": (s["yaw"] - 50) * 3.3,
                "gas_adc": int(180 + 40 * (1 + math.sin(t / 7))),
                "geiger_cps": counts[-1],
                "geiger_counts_60s": sum(counts),
                "battery_v": round(12.4 - 0.004 * t, 2),
                "rx_time": time.time(),
            }
            self.frames += 1
            time.sleep(0.1)

    def stop(self) -> None:
        self._stop.set()


# ---------------------------------------------------------------------------
# Bridge
# ---------------------------------------------------------------------------
class Bridge:
    def __init__(self, cfg: configparser.ConfigParser, simulate: bool):
        fb = cfg["firebase"]
        self.db = FirebaseRTDB(fb["database_url"], fb["api_key"], fb["email"], fb["password"])
        self.base = f"drones/{cfg['drone'].get('id', 'drone1')}"
        self.timeout = cfg["drone"].getfloat("command_timeout", 1.0)
        self.poll_hz = cfg["drone"].getfloat("poll_hz", 10.0)
        pins = {k: cfg["pins"].getint(k, v) for k, v in DEFAULT_PINS.items()} if "pins" in cfg else DEFAULT_PINS
        self.out = PulseOutput(pins, simulate)
        ser = cfg["serial"] if "serial" in cfg else {}
        self.telemetry = TelemetryReader(ser.get("port", "/dev/serial0"), int(ser.get("baud", 115200)), simulate)
        sens = cfg["sensors"] if "sensors" in cfg else {}
        self.r0 = float(sens.get("mq2_r0_kohm", 1.0))
        self.rl = float(sens.get("mq2_rl_kohm", 1.0))
        self.tube = sens.get("geiger_tube", "SBM-20")
        self.last_seq = None
        self.last_fresh = 0.0
        self.running = True

    def physical(self, f: dict) -> dict:
        rad = sensors.geiger_dose(f["geiger_counts_60s"], self.tube)
        return {
            **{k: v for k, v in f.items() if k != "rx_time"},
            "gas_lpg_ppm": round(sensors.mq2_ppm(f["gas_adc"], self.r0, "LPG", self.rl), 1),
            "gas_co_ppm": round(sensors.mq2_ppm(f["gas_adc"], self.r0, "CO", self.rl), 1),
            "gas_smoke_ppm": round(sensors.mq2_ppm(f["gas_adc"], self.r0, "smoke", self.rl), 1),
            "radiation_cpm": round(rad.cpm, 1),
            "radiation_usv_h": round(rad.usv_h, 4),
            "radiation_sigma_usv_h": round(rad.sigma_usv_h, 4),
            "geiger_tube": self.tube,
            "time": {".sv": "timestamp"},     # Firebase server timestamp
        }

    def run(self) -> None:
        self.db.sign_in()
        log.info("signed in to Firebase, path /%s", self.base)
        self.telemetry.start()
        period = 1.0 / self.poll_hz
        last_hist = 0.0
        link_up = False
        while self.running:
            t_loop = time.time()
            # ---- commands --------------------------------------------------
            try:
                cmd = self.db.get(f"{self.base}/command")
                if cmd and cmd.get("seq") != self.last_seq:
                    self.last_seq = cmd.get("seq")
                    self.last_fresh = time.time()
                    sticks = {k: float(cmd.get(k, NEUTRAL[k])) for k in NEUTRAL}
                    self.out.write(sticks)
                    self.telemetry.sim_sticks = sticks
            except Exception as e:  # network error: watchdog below handles it
                log.warning("command read failed: %s", e)

            fresh = (time.time() - self.last_fresh) < self.timeout
            if not fresh and link_up:
                log.error("command link lost > %.1f s -> pulses OFF (flight-controller fail-safe)", self.timeout)
                self.out.off()
            link_up = fresh

            # ---- telemetry -------------------------------------------------
            f = self.telemetry.latest
            if f and time.time() - f["rx_time"] < 1.0:
                try:
                    data = self.physical(f)
                    data["link_ok"] = fresh
                    data["pulses_us"] = self.out.last
                    self.db.put(f"{self.base}/telemetry/latest", data)
                    if time.time() - last_hist >= 1.0:
                        self.db.post(f"{self.base}/telemetry/history", data)
                        last_hist = time.time()
                except Exception as e:
                    log.warning("telemetry upload failed: %s", e)

            time.sleep(max(0.0, period - (time.time() - t_loop)))

    def shutdown(self, *_):
        self.running = False
        self.telemetry.stop()
        self.out.close()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default="config.ini")
    ap.add_argument("--simulate", action="store_true", help="no GPIO/UART, synthetic telemetry")
    ap.add_argument("--calibrate-mq2", type=int, metavar="ADC",
                    help="print R0 for an ADC value read in clean air and exit")
    args = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")

    if args.calibrate_mq2 is not None:
        print(f"R0 = {sensors.mq2_calibrate_r0(args.calibrate_mq2):.3f} kOhm  -> put it in [sensors] mq2_r0_kohm")
        return 0

    cfg = configparser.ConfigParser()
    if not cfg.read(args.config):
        log.error("config file %s not found (copy config.example.ini)", args.config)
        return 1
    bridge = Bridge(cfg, args.simulate)
    signal.signal(signal.SIGINT, bridge.shutdown)
    signal.signal(signal.SIGTERM, bridge.shutdown)
    try:
        bridge.run()
    finally:
        bridge.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
