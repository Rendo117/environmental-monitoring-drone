# Flight-controller firmware (ATmega328P / Arduino Uno)

* `flight_controller/flight_controller.ino`: main sketch
* `flight_controller/config.h`: every tunable value (sensors on/off, PID gains, IMU signs, fail-safe, telemetry)
* `host_test/`: compiles the sketch on a PC against a mocked Arduino API and runs arming → flight → link-loss

Upload with the Arduino IDE (board **Arduino Uno**). Keep the aircraft still during the first 8 s (gyro calibration, LED on).

| Pin | Function |
|---|---|
| D8, D9, D10, D11 | roll, pitch, throttle, yaw pulses from the Raspberry Pi |
| D12 | Geiger pulse (active LOW) |
| D4–D7 | ESC1–ESC4 |
| A4/A5 | MPU-6050 I²C |
| A3 | MQ-2 |
| A0 | battery divider |
| TX | telemetry to the Pi (through a 1 k/2 k divider) |

Telemetry frame (115 200 Bd, 10 Hz):
`$T,<state>,<failsafe>,<roll×10>,<pitch×10>,<yawrate×10>,<gas ADC>,<counts last s>,<counts last 60 s>,<Vbat cV>*<XOR hex>`
