# Bench set-up, calibration and first-flight checklist

> ⚠ **Do every step with the propellers removed** until step 6.

## 1. Flash the flight controller
1. Arduino IDE 2.x → open `firmware/flight_controller/flight_controller.ino` (keep `config.h` in the same folder).
2. Board **Arduino Uno**, upload. LED 13 stays on for ~8 s while the gyro is calibrated, so keep the frame still and level.
3. Host-side test (no hardware): `cd firmware/host_test && g++ -std=gnu++17 -I. -x c++ main.cpp -o fc_test && ./fc_test`

## 2. Sign check of the IMU (fixes errata A1)
Open the Serial Monitor at 115 200 Bd. Telemetry frames look like
`$T,<status>,<fs>,<roll×10>,<pitch×10>,<yawrate×10>,<gas>,<cps>,<cpm>,<vbat>*CS`.

| Action | Expected | If wrong, flip |
|---|---|---|
| lift the **left** side | roll field **positive** | `GYRO_X_SIGN` **and** `ACC_Y_SIGN` |
| lift the **nose** | pitch field **positive** | `GYRO_Y_SIGN` **and** `ACC_X_SIGN` |
| rotate the nose to the **right** | yaw-rate field **positive** while turning | `GYRO_Z_SIGN` |
| tilt and hold 10 s | the angle stays put (no slow drift back to 0) | the ACC sign of that axis only |

Always flip the gyro and accelerometer sign of an axis **together**, unless the last test shows
that only the accelerometer disagrees.

## 3. Motor direction and mixer check (props off)
1. Arm from the ground station: **ARM** (throttle 0 + yaw 0, then yaw 50).
2. Throttle 15 %. All four motors spin. Check the directions: M1 front-left **CW**, M2 front-right **CCW**,
   M3 rear-left **CCW**, M4 rear-right **CW**. To reverse a motor, swap any two of its three wires.
3. Tilt the frame by hand to the **left side down**. The **left** motors (M1, M3) must speed up.
4. Tilt **nose down**. The **front** motors (M1, M2) must speed up.
5. Yaw the frame **to the right** by hand. M1 and M4 (CW) must speed up.
6. **DISARM** (throttle 0 + yaw 100). All motors stop.

## 4. ESC calibration
Calibrate all four ESCs to the same 1000–2000 µs range with a servo tester or the usual
"full throttle at power-up" procedure from the ESC manual.

## 5. Payload sensors
* **MQ-2:** run it powered for 24 h once (burn-in), then 3 min before each use. In clean outdoor air, read
  the `gas` field (ADC) and run `python3 raspberry-pi/bridge.py --calibrate-mq2 <ADC>`. Put the printed
  R0 into `config.ini`.
* **Geiger:** with a high-impedance probe (≥ 1 GΩ divider) set the 555 frequency/duty so the tube runs
  in the middle of its plateau (SBM-20: ~400 V). In the open air, expect 15–30 CPM (≈ 0.1 µSv/h).
  Set `geiger_tube` in `config.ini`.
* **Battery:** compare the `vbat` field with a multimeter. If needed, adjust `BATT_SCALE` / `BATT_OFFSET` in `config.h`.

## 6. First flight
* Open field, away from people, line of sight, light wind, fully charged pack.
* Check the fail-safe on the ground first: arm, throttle 20 % (props **on**, aircraft tied down),
  then stop `bridge.py`. The motors must spool down within ~1.5 s and disarm.
* Fly the first flights from the ground station with **Wi-Fi on the same network** to keep latency low.
  Try the internet/4G path only after that.
