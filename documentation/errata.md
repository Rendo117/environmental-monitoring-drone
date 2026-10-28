# Errata: corrections to the 2022 thesis

This is a review of *"Dronă de colectare a datelor despre mediul înconjurător în situații de urgență"*
(C. M. Chirea, University of Craiova, July 2022). Page numbers are the printed page numbers.
Severity: 🔴 affects safety/function · 🟠 technically wrong · 🟡 wording/typo.

## A. Firmware (Annex A, "Codul C de control a dronei")

| # | Sev. | Original | Problem | Fix in this repository |
|---|---|---|---|---|
| A1 | 🔴 | `gyro_angle[Y] += -gyro_raw[Y]/…` and `angular_motions[PITCH] = … + gyro_raw[Y]` | The pitch angle and the pitch rate have opposite signs, so the self-levelling loop has a pole at $s=+5$ s⁻¹. The aircraft flips on pitch in under a second (see simulation). | One sign per axis (`GYRO_*_SIGN`, `ACC_*_SIGN`). Rate = d(angle)/dt by construction. |
| A2 | 🔴 | `isRULARE()` defined but never called | No arming logic. The ESCs get ≥ 1100 µs (props spinning) as soon as the loop starts. | `updateStatus()` runs every loop. Motors stay at 1000 µs unless STARTED. |
| A3 | 🔴 | — | No reaction to losing the link. The last pulse widths stay valid forever. | 250 ms pulse timeout → level attitude, throttle ramp-down, disarm. The RPi also has a 1 s command watchdog. |
| A4 | 🔴 | `ISR(PCINT0_vect)` defined **twice** (receiver and Geiger) | Does not compile when `GEYGER_SENSOR_PRESENT` = 1. | One ISR handles PB0–PB4. Falling-edge counting for the Geiger tube on PCINT4 (`PCMSK0` bit 4 is now enabled). |
| A5 | 🟠 | `TCNT1 = 31250` with prescaler 256, overflow interrupt | The period is (65 536 − 31 250)·16 µs = **0.549 s**, not 1 s. | Timer1 in CTC mode, `OCR1A = 62499` → exactly 1.000 Hz. |
| A6 | 🟠 | `Serial.Write(...)` inside the timer ISR | Capital `W` does not compile. Serial output from an ISR can also block. | The ISR only stores counts in a 60-s ring buffer. Telemetry is sent from `loop()`. |
| A7 | 🟠 | `void citireSenzorGaz() { return analogRead(A3); }` and `#elif` without condition | A `void` function returns a value, and `#elif` needs an expression. Neither compiles. The gas value was never used. | `readPayload()` stores `gas_adc`, which goes out in the telemetry frame. |
| A8 | 🟠 | `interruptPinTemp`, `interruptPinSlanost`, `timer1_counter`, `ticks` | Undeclared identifiers (copied from another project). | Removed or declared. |
| A9 | 🟠 | `Serial.write(int(miscari_unghiulare[YAW]))` every loop | Sends one raw binary byte at 250 Hz, which corrupts any serial protocol and wastes loop time. | Removed. A structured `$T,…*CS` frame is sent at 10 Hz. |
| A10 | 🟠 | `acc_total_vector` is `long`; `if (abs(acc_raw[X]) < acc_total_vector) acc_angle[X] = asin(acc_raw[Y]/…)` | Integer truncation can make the ratio > 1, which gives NaN. The guard checks the wrong axis. | `float` norm and `constrain(ratio, -1, 1)` on the same axis. |
| A11 | 🟠 | `delay(0.1)` | `delay()` takes integer ms, so this is `delay(0)`. | Removed. |
| A12 | 🟠 | Yaw "centred" test `== 1500` | A measured pulse is almost never exactly 1500 µs, so arming could hang in STARTING. | Tolerance ±50 µs. |
| A13 | 🟠 | Mixer outputs computed in `unsigned long` | A negative intermediate value wraps to ~4·10⁹ before clamping. | Computed in `float`, then clamped to 1100–2000 µs. |
| A14 | 🟠 | Volatile pulse widths read directly in `loop()` | 16/32-bit reads are not atomic on AVR, so a value can be torn by the ISR. | Copied with interrupts disabled. |
| A15 | 🟡 | Comment `ACCEL_CONFIG 0x10 // ±500°/s` | 0x10 means **±8 g** (4096 LSB/g). | Comment corrected. |
| A16 | 🟠 | Initial angle = one accelerometer sample | Vibration noise becomes a permanent tilt offset. On a multirotor the accelerometer cannot correct it later. | Initialised from the mean of the 2000 calibration samples. |
| A17 | 🟡 | `compensateBatteryDrop()` never called | Dead code. | Called when STARTED and a battery is detected. |
| A18 | 🟡 | Mixed Romanian/English identifiers, `GEYGER` | Readability. | English identifiers, `GEIGER`. |

## B. Raspberry Pi script (§4.3)

| # | Sev. | Problem | Fix |
|---|---|---|---|
| B1 | 🔴 | **Channel order mismatch.** Java writes `pitch|roll|yaw|throttle`, but the Pi parses `num[0]=yaw, num[1]=roll, num[2]=pitch`. Pitch commands move yaw and the reverse. | Named JSON fields `{roll, pitch, yaw, throttle}`. |
| B2 | 🔴 | No authentication. The database URL in the thesis is world-writable, so anyone could send flight commands. | Firebase Auth (e-mail/password) plus security rules (`firebase/database.rules.json`). |
| B3 | 🟠 | `RPi.GPIO` software PWM has tens of µs of jitter, about 3–5 % of stick travel. | `pigpio` DMA-timed servo pulses (< 5 µs jitter). |
| B4 | 🟠 | `GPIO.setmode(GPIO.BOARD)` with pins 12/13/18/19. These are **physical** pins (BCM 18/27/24/10). Only BCM18 supports hardware PWM. | Documented. BCM numbers in `config.ini`. |
| B5 | 🟠 | The str(dict) parser reads digits between `:` and `,`, which breaks with nested keys, negative numbers and Firebase push IDs. | JSON. |
| B6 | 🟠 | Throttle printed as `num[3]/10` but sent as `num[3]`. | Removed. |
| B7 | 🟠 | No telemetry upload, although the thesis says measurements are sent to the database. | `telemetry/latest` + 1 Hz `history`. |

## C. Java application (§4.2)

| # | Sev. | Problem | Fix |
|---|---|---|---|
| C1 | 🔴 | Credentials hard-coded (`user` / `drone`), compared in plain text in the client. This gives no security. | Real authentication through Firebase Auth. Nothing is stored in the code. |
| C2 | 🟠 | Every key press writes `log.txt`, starts a `.bat`, and that starts a new Python interpreter. Latency is 0.3–1 s per command and no ordering is guaranteed. | Direct HTTPS `PUT` from Java (`java.net.http`). |
| C3 | 🟠 | Absolute Windows paths (`D:\projects\…`) in three different forms (`\src…`, `src…`, `D:\…`). | No file system use. |
| C4 | 🟠 | Spinner default 0 for roll/pitch/yaw is 1000 µs, which is a **full left/down stick**. Arming sends extreme commands. | Default 50 (1500 µs centre). |
| C5 | 🟠 | Keyboard works only when the radio button has focus. `VK_DOWN` and the others lack the `connectionRequired` check. | Window-wide key bindings with one check. |
| C6 | 🟠 | Gas/radiation fields in the GUI are never filled. | Telemetry tab with values, uncertainty, plots and CSV export. |
| C7 | 🟡 | The 9-fold copy-paste of the same send block (~400 lines). | One `sendCommand()`. |

## D. Hardware

| # | Sev. | Item | Correction |
|---|---|---|---|
| D1 | 🔴 | Arduino TX (5 V) → Raspberry Pi RX | Pi GPIOs are **3.3 V only**. Use a 1 k/2 k divider (added). |
| D2 | 🔴 | RPi 4 powered from the ESC BEC (Fritzing Fig. 1) | Linear ESC BECs deliver about 1–2 A. A Pi 4 needs a 5 V / 3 A supply, otherwise it browns out and reboots in flight. Use a dedicated UBEC. |
| D3 | 🟠 | Geiger schematic (Fig. 8): D1, D2 labelled **2N2222** | 2N2222 is a transistor. Use fast HV diodes such as UF4007. |
| D4 | 🟠 | Geiger output taken from the base of Q2 | The base swing is only ~0.6 V. Take the output from the **collector** (active-LOW with R10 pull-up). |
| D5 | 🟠 | "4.7m" resistors | In SPICE notation "m" means milli. The anode resistor is 4.7 **MΩ**. |
| D6 | 🟠 | Q1 IRF220 driven from a 5 V 555 | $V_{GS(th)}$ is up to 4 V, so the device is not fully enhanced. Use a logic-level MOSFET (IRLZ44N), a 100 Ω gate resistor and a 10 k pull-down. |
| D7 | 🟠 | Tattu 2305 **2450 KV** motors listed, but the photo shows an F450-class frame with ~10″ props | 2450 KV motors are for 5″ props. On 10″ props they would draw far more than the 30 A ESC rating. For F450 use 920–1000 KV (2212) with 10×4.5″ props. Check which motors were actually fitted. |
| D8 | 🟡 | R7 650 Ω | Not an E12 value. Use 680 Ω. |

## E. Text

| Page | Sev. | Original | Correction |
|---|---|---|---|
| x (contents), 18 | 🟡 | "Modelul matematic a dornei", "Structura donei" | "al dronei", "dronei" |
| 3 | 🟡 | "Raspberry Pi 4 Model B este cel mai nou model" | True in 2022. Raspberry Pi 5 was released in October 2023. |
| 4 | 🟡 | BCM2835 / ARM1176 paragraph | Describes the **first-generation** Pi, not the Pi 4 used. Can be removed. |
| 5 | 🟠 | "Viteza de 20 MIPS" | 20 MIPS at 20 MHz. The Uno runs at **16 MHz** (16 MIPS). |
| 6 | 🟠 | "Plăcuța se poate alimenta la tensiuni între 1.8 V și 5.5 V" | That is the **ATmega328P chip** range. The Uno **board** takes 7–12 V on Vin/jack or 5 V on USB. |
| 6–7 | 🟠 | "MPU6050 – Accelerometru" | It is a 6-axis **IMU** (3-axis gyroscope + 3-axis accelerometer). |
| 7 | 🟡 | VCC "3.3 V" | The GY-521 module has an on-board regulator and accepts 3.3–5 V. |
| 10 | 🟠 | "Un BEC liniar folosește practic un rezistor" | A linear BEC uses a **linear regulator** (pass transistor). The excess power $(V_{in}-5)\,I$ is dissipated as heat. |
| 11 | 🟡 | "cele mai bune și mai puternice motoare … făcute vreodată" | Marketing text from the vendor. Replace with specifications. |
| 12 | 🟠 | "Când particulele ionizante străbat tubul, circuitul se închide" | Ionising radiation ionises the fill gas, and the high field creates a **Townsend avalanche**. A short current pulse flows; the circuit does not "close". |
| 12 | 🟡 | "IRF220" | See D6. |
| 13 | 🟡 | "Limbajul C este independent de platformă" | The *source* is portable. Compiled binaries are not (the thesis says this later). |
| 15 | 🟡 | Java "error-freedom", "programs … not at risk of infection" | Over-statements. Java is memory-safe, but not free of bugs or vulnerabilities. |
| 19 | 🟠 | "putem controla doar unghiurile φ, θ și ψ" | The inputs are the thrust $T$ **and** the three torques (see math-model §4). |
| 20 | 🟠 | $q = [-\xi;\ \eta]$ | $q=[\xi;\ \eta]$ |
| 21 | 🟠 | $\theta \neq (2k-1)\phi/2$ | $\theta \neq (2k-1)\pi/2$ |
| 22 | 🟠 | $\tau_\psi = \sum\tau_{M_i}$ | $\tau_\psi=\sum(-1)^{i+1}\tau_{M_i}$ |
| 22 | 🟡 | "rotație", "înclinare", "rotire" used interchangeably | roll = *ruliu*, pitch = *tangaj*, yaw = *girație* (standard Romanian aeronautical terms). |
| 26 | 🟠 | "un timer pentru a detecta într-o secundă" | See A5: the configured period was 0.549 s. |
| 44 | 🟡 | "factorii … se pot alege în mod experimental" | Now supported by the stability analysis in math-model §7.3. |
| 47 | 🟡 | Reference [3] "2405KV" | 2450 KV |
| — | 🟡 | "esdte", "unchiulare", "sirecţii", "viarea", "peverticală", "ecceleraţiei", "senzotului", "prentru", "înntre", "v-oi", "Arrrow" | este, unghiulare, direcţii, iar, pe verticală, acceleraţiei, senzorului, pentru, între, voi, Arrow |
