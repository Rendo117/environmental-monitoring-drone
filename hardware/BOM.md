# Bill of materials

| Qty | Part | Notes |
|---|---|---|
| 1 | Raspberry Pi 4 Model B (2 GB or more) + micro-SD 16 GB | companion computer, Raspberry Pi OS Lite 64-bit |
| 1 | Arduino Uno R3 (ATmega328P) | flight controller |
| 1 | GY-521 module (MPU-6050) | IMU, I²C 0x68 |
| 1 | MQ-2 module (FC-22 type, RL = 1 kΩ) | gas sensor |
| 1 | Geiger board: NE555, IRLZ44N, 6 V:117 V transformer, 2× UF4007, 2× 10 nF/1 kV, 4.7 MΩ, 2N2222, resistors per `schematics/geiger.svg` | or a ready-made "RadiationD" kit |
| 1 | GM tube SBM-20 or J305 | |
| 1 | F450 frame (or similar 450 mm) | |
| 4 | Brushless motor 2212 920–1000 KV | *thesis listed Tattu 2305 2450 KV, see errata D7* |
| 4 | Propeller 10×4.5″ (2 CW + 2 CCW) | |
| 4 | ESC 30 A (OPTO, SimonK/BLHeli) | |
| 1 | LiPo 3S 11.1 V 2200–3300 mAh + XT60 | |
| 1 | UBEC 5 V / 3 A | powers Pi, Uno, sensors |
| 1 | 1N4001, 1.5 kΩ, 1 kΩ | battery sense |
| 1 | 1 kΩ, 2 kΩ | UART divider |
| 1 | 4G USB modem *(optional)* | for operation beyond Wi-Fi range |
| — | *recommended upgrade:* BMP280 barometer | enables a controlled fail-safe descent |
