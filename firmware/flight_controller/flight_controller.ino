/*
 * flight_controller.ino
 * ---------------------------------------------------------------------------
 * Flight controller + payload acquisition for the environmental-monitoring
 * quadcopter (ATmega328P / Arduino Uno @ 16 MHz).
 *
 *   Original: C. M. Chirea, "Dronă de colectare a datelor despre mediul
 *             înconjurător în situații de urgență", Univ. of Craiova, 2022.
 *   Revision 2026: bug fixes, arming state machine actually used, fail-safe,
 *             atomic reads of ISR data, single merged PCINT0 ISR (the original
 *             defined ISR(PCINT0_vect) twice), 1 Hz Timer1 in CTC mode
 *             (the original pre-load gave 0.549 s), Geiger 60 s rolling CPM,
 *             gas sensor read + telemetry frame with checksum for the RPi.
 *
 * Hardware map (see hardware/wiring.md):
 *   D8  (PB0, PCINT0)  <- RPi phys pin 13  ROLL     channel 1
 *   D9  (PB1, PCINT1)  <- RPi phys pin 18  PITCH    channel 2
 *   D10 (PB2, PCINT2)  <- RPi phys pin 19  THROTTLE channel 3
 *   D11 (PB3, PCINT3)  <- RPi phys pin 12  YAW      channel 4
 *   D12 (PB4, PCINT4)  <- Geiger pulse (active LOW, open collector + pull-up)
 *   D4..D7 (PD4..PD7)  -> ESC1..ESC4 signal
 *   A4/A5 (SDA/SCL)    <-> MPU-6050 (0x68)
 *   A3                 <- MQ-2 AOUT
 *   A0                 <- battery divider
 *   TX (D1)            -> 1k/2k divider -> RPi RX (GPIO15, phys pin 10)
 *
 * Sign convention: roll + left wing up, pitch + nose up, yaw + nose right.
 *
 * Motor layout ("X" configuration, seen from above, nose up):
 *        ESC1 (FL, CW)      ESC2 (FR, CCW)
 *                   \        /
 *                    [ body ]
 *                   /        \
 *        ESC3 (RL, CCW)     ESC4 (RR, CW)
 *   Derived from the mixer signs below; VERIFY WITH PROPELLERS REMOVED.
 *
 * Structure derived from lobodol/drone-flight-controller (MIT), itself based on
 * YMFC-AL by Joop Brokking. See THIRD_PARTY_NOTICES.md.
 *
 * SPDX-License-Identifier: MIT
 */
#include <Wire.h>
#include "config.h"

// ----------------------------- indices -------------------------------------
enum { CH1 = 0, CH2 = 1, CH3 = 2, CH4 = 3 };   // receiver channels (D8..D11)
enum { YAW = 0, PITCH = 1, ROLL = 2, THROTTLE = 3 };
enum { X = 0, Y = 1, Z = 2 };
enum { STOPPED = 0, STARTING = 1, STARTED = 2 };

#define MPU_ADDRESS 0x68

// ----------------------------- receiver (ISR) ------------------------------
volatile uint8_t  prev_state[4]   = {0, 0, 0, 0};
volatile uint16_t pulse_len[4]    = {PULSE_MID, PULSE_MID, PULSE_MIN, PULSE_MID};
volatile uint32_t rise_time[4];
volatile uint32_t last_seen_ms[4] = {0, 0, 0, 0};
uint8_t  mode_mapping[4];
uint16_t pulse[4];               // atomic copy used by the control loop

// ----------------------------- Geiger (ISR) --------------------------------
#if GEIGER_SENSOR_PRESENT
volatile uint8_t  geiger_prev    = 1;
volatile uint16_t geiger_ticks   = 0;     // counts in the current 1 s gate
volatile uint8_t  cps_ring[60];           // last 60 one-second gates
volatile uint8_t  cps_idx        = 0;
volatile uint16_t cpm_rolling    = 0;     // sum of the ring = counts per minute
volatile uint8_t  cps_last       = 0;
#endif

// ----------------------------- IMU -----------------------------------------
int16_t gyro_raw[3] = {0, 0, 0};
int32_t gyro_offset[3] = {0, 0, 0};
float   gyro_angle[3] = {0, 0, 0};
int16_t acc_raw[3] = {0, 0, 0};
float   acc_angle[3] = {0, 0, 0};
int16_t temperature_raw;
float   angular_motions[3] = {0, 0, 0};     // filtered body rates [deg/s]: yaw, pitch, roll
float   measures[3] = {0, 0, 0};            // filtered angles [deg]: (yaw rate), pitch, roll
bool    initialized = false;

// ----------------------------- PID -----------------------------------------
float pid_set_points[3] = {0, 0, 0};
float errors[3]         = {0, 0, 0};
float delta_err[3]      = {0, 0, 0};
float error_sum[3]      = {0, 0, 0};
float previous_error[3] = {0, 0, 0};

// ----------------------------- outputs / state -----------------------------
uint32_t loop_timer;
uint32_t now_us, difference;
uint16_t esc_pulse[4] = {PULSE_MIN, PULSE_MIN, PULSE_MIN, PULSE_MIN};
uint8_t  status = STOPPED;
bool     failsafe = false;
bool     link_ok = false;                    // valid pulses on every channel recently
uint16_t failsafe_throttle = PULSE_MIN;
int16_t  battery_cv = BATT_FULL_CV;          // battery voltage [0.01 V]
uint16_t gas_adc = 0;
uint8_t  telemetry_count = 0;


// ----------------------------- prototypes ----------------------------------
// (the Arduino IDE generates these automatically; explicit for other toolchains)
void readReceiver(); void configureChannelMapping(); void updateStatus();
void writeMpuRegister(uint8_t reg, uint8_t value); void setupMpu6050Registers();
void readSensor(); void calibrateMpu6050(); void calculateGyroAngles();
void calculateAccelerometerAngles(); void calculateAngles(); void resetGyroAngles();
float calculateSetPoint(float angle, int channel_pulse);
float calculateYawSetPoint(int yaw_pulse, int throttle_pulse);
void calculateSetPoints(); void calculateErrors(); void pidController();
void resetPidController(); void stopAll(); float minMax(float v, float lo, float hi);
bool isBatteryConnected(); void compensateBatteryDrop(); void readPayload();
void sendTelemetry(); void applyMotorSpeed();

// ============================================================================
void setup() {
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);                    // LED on during start-up/calibration
  Serial.begin(SERIAL_BAUD);

  Wire.begin();
  TWBR = 12;                                 // I2C 400 kHz: f = 16 MHz / (16 + 2*12)

  DDRD |= B11110000;                         // D4..D7 outputs (ESC signals)
  DDRB &= B11100000;                         // D8..D12 inputs
  PORTB |= B00010000;                        // pull-up on D12 (Geiger, open collector)

  setupMpu6050Registers();
  calibrateMpu6050();                        // also keeps ESCs armed-silent (1000 us)
  configureChannelMapping();

  noInterrupts();
  // Pin-change interrupts on PB0..PB4 (D8..D12)
  PCICR  |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT0) | (1 << PCINT1) | (1 << PCINT2) | (1 << PCINT3);
#if GEIGER_SENSOR_PRESENT
  PCMSK0 |= (1 << PCINT4);
  // Timer1, CTC mode, prescaler 256: f = 16e6 / (256 * (62499 + 1)) = 1.000 Hz
  TCCR1A = 0;
  TCCR1B = (1 << WGM12) | (1 << CS12);
  OCR1A  = 62499;
  TCNT1  = 0;
  TIMSK1 |= (1 << OCIE1A);
#endif
  interrupts();

#if BATTERY_SENSE_PRESENT
  battery_cv = (int16_t)((analogRead(A0) + BATT_OFFSET) * BATT_SCALE);
#endif
  loop_timer = micros();
  digitalWrite(13, LOW);
}

// ============================================================================
void loop() {
  readReceiver();          // atomic copy of the pulse widths + fail-safe check
  readSensor();            // MPU-6050 raw data
  calculateAngles();       // complementary filter
  updateStatus();          // arming state machine (was never called in 2022)
  calculateSetPoints();
  calculateErrors();
  if (status == STARTED) {
    pidController();
#if BATTERY_SENSE_PRESENT
    compensateBatteryDrop();
#endif
  } else {
    stopAll();
  }
  readPayload();
  sendTelemetry();
  applyMotorSpeed();       // waits for the 4 ms tick, then generates the ESC pulses
}

// ----------------------------------------------------------------------------
// Receiver
// ----------------------------------------------------------------------------
void readReceiver() {
  uint32_t oldest;
  noInterrupts();                            // 16/32-bit volatile reads are not atomic on AVR
  for (uint8_t i = 0; i < 4; i++) pulse[i] = pulse_len[i];
  oldest = last_seen_ms[0];
  for (uint8_t i = 1; i < 4; i++) if (last_seen_ms[i] < oldest) oldest = last_seen_ms[i];
  interrupts();

  for (uint8_t i = 0; i < 4; i++) pulse[i] = constrain(pulse[i], PULSE_MIN, PULSE_MAX);

  bool lost = (oldest == 0) || (millis() - oldest) > FAILSAFE_TIMEOUT_MS;
  link_ok = !lost;
  if (lost && !failsafe && status == STARTED) {
    failsafe = true;
    failsafe_throttle = pulse[mode_mapping[THROTTLE]];
  }
  if (!lost) failsafe = false;

  if (failsafe) {                            // level attitude, centred yaw, ramp throttle down
    pulse[mode_mapping[ROLL]]  = PULSE_MID;
    pulse[mode_mapping[PITCH]] = PULSE_MID;
    pulse[mode_mapping[YAW]]   = PULSE_MID;
    static uint8_t ramp_div = 0;
    if (++ramp_div >= FAILSAFE_RAMP_DIV) {
      ramp_div = 0;
      if (failsafe_throttle > ESC_IDLE) failsafe_throttle--;
    }
    pulse[mode_mapping[THROTTLE]] = failsafe_throttle;
    if (failsafe_throttle <= ESC_IDLE && status == STARTED) {
      status = STOPPED;                      // landed (or out of options): disarm
    }
  }
}

void configureChannelMapping() {
  mode_mapping[YAW]      = CH4;
  mode_mapping[PITCH]    = CH2;
  mode_mapping[ROLL]     = CH1;
  mode_mapping[THROTTLE] = CH3;
}

// ----------------------------------------------------------------------------
// Arming state machine
//   STOPPED  --(yaw left & throttle low)-->  STARTING
//   STARTING --(yaw centred & throttle low)--> STARTED   (PID + angles reset)
//   STARTED  --(yaw right & throttle low)--> STOPPED
// ----------------------------------------------------------------------------
void updateStatus() {
  uint16_t yaw = pulse[mode_mapping[YAW]];
  uint16_t thr = pulse[mode_mapping[THROTTLE]];

  if (status == STOPPED && yaw <= ARM_LOW && thr <= ARM_LOW && link_ok) {
    status = STARTING;
  }
  if (status == STARTING && abs((int)yaw - PULSE_MID) < ARM_CENTER_TOL && thr <= ARM_LOW) {
    status = STARTED;
    resetPidController();
    resetGyroAngles();
  }
  if (status == STARTED && yaw >= ARM_HIGH && thr <= ARM_LOW) {
    status = STOPPED;
  }
  digitalWrite(13, status == STARTED ? HIGH : LOW);
}

// ----------------------------------------------------------------------------
// MPU-6050
// ----------------------------------------------------------------------------
void writeMpuRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void setupMpu6050Registers() {
  writeMpuRegister(0x6B, 0x00);  // PWR_MGMT_1   : wake up, internal 8 MHz clock
  writeMpuRegister(0x1B, 0x08);  // GYRO_CONFIG  : +/-500 deg/s  -> 65.5 LSB/(deg/s)
  writeMpuRegister(0x1C, 0x10);  // ACCEL_CONFIG : +/-8 g        -> 4096 LSB/g
  writeMpuRegister(0x1A, 0x03);  // CONFIG       : DLPF ~44 Hz (accel) / 42 Hz (gyro)
}

void readSensor() {
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x3B);                          // ACCEL_XOUT_H, then 14 consecutive bytes
  Wire.endTransmission();
  Wire.requestFrom(MPU_ADDRESS, 14);
  uint32_t t0 = micros();
  while (Wire.available() < 14) {
    if (micros() - t0 > 2000) return;        // I2C timeout: keep previous sample
  }
  acc_raw[X]      = Wire.read() << 8 | Wire.read();
  acc_raw[Y]      = Wire.read() << 8 | Wire.read();
  acc_raw[Z]      = Wire.read() << 8 | Wire.read();
  temperature_raw = Wire.read() << 8 | Wire.read();
  gyro_raw[X]     = Wire.read() << 8 | Wire.read();
  gyro_raw[Y]     = Wire.read() << 8 | Wire.read();
  gyro_raw[Z]     = Wire.read() << 8 | Wire.read();
}

void calibrateMpu6050() {
  const int max_samples = 2000;              // ~8 s, the quadcopter must stay still
  int32_t acc_sum[3] = {0, 0, 0};
  for (int i = 0; i < max_samples; i++) {
    readSensor();
    acc_sum[X] += acc_raw[X]; acc_sum[Y] += acc_raw[Y]; acc_sum[Z] += acc_raw[Z];
    gyro_offset[X] += gyro_raw[X];
    gyro_offset[Y] += gyro_raw[Y];
    gyro_offset[Z] += gyro_raw[Z];
    PORTD |= B11110000;                      // 1000 us pulse keeps the ESCs armed but silent
    delayMicroseconds(1000);
    PORTD &= B00001111;
    delay(3);
  }
  gyro_offset[X] /= max_samples;
  gyro_offset[Y] /= max_samples;
  gyro_offset[Z] /= max_samples;

  // Initial attitude from the AVERAGED accelerometer (noise / sqrt(2000)) instead of
  // a single vibrating sample: on a multirotor the accelerometer cannot correct a tilt
  // error later (in hover the specific force stays on the body z axis), so a bad
  // first estimate becomes a permanent drift-producing offset.
  for (uint8_t a = 0; a < 3; a++) acc_raw[a] = acc_sum[a] / max_samples;
  calculateAccelerometerAngles();
  resetGyroAngles();
  initialized = true;
}

void calculateGyroAngles() {
  gyro_raw[X] = (gyro_raw[X] - gyro_offset[X]) * GYRO_X_SIGN;   // roll rate   [LSB]
  gyro_raw[Y] = (gyro_raw[Y] - gyro_offset[Y]) * GYRO_Y_SIGN;   // pitch rate  [LSB]
  gyro_raw[Z] = (gyro_raw[Z] - gyro_offset[Z]) * GYRO_Z_SIGN;   // yaw rate    [LSB]

  // integrate rate: angle += omega * dt  with omega = raw/SSF, dt = 1/f.
  // Same sign as the rate used by the PID (fixed: 2022 used -gyro_raw[Y] here).
  gyro_angle[X] += gyro_raw[X] / (LOOP_FREQ_HZ * SSF_GYRO);
  gyro_angle[Y] += gyro_raw[Y] / (LOOP_FREQ_HZ * SSF_GYRO);

  // the tilt vector rotates in the body frame when the aircraft yaws by d_psi:
  //   pitch' = pitch - roll*sin(d_psi) ,  roll' = roll + pitch*sin(d_psi)
  float s = sin(gyro_raw[Z] * (PI / (LOOP_FREQ_HZ * SSF_GYRO * 180.0f)));
  gyro_angle[Y] -= gyro_angle[X] * s;
  gyro_angle[X] += gyro_angle[Y] * s;
}

void calculateAccelerometerAngles() {
  float ax = acc_raw[X] * ACC_X_SIGN, ay = acc_raw[Y] * ACC_Y_SIGN, az = acc_raw[Z];
  float total = sqrt(ax * ax + ay * ay + az * az);  // float: no truncation (was 'long')
  if (total < 1.0f) return;
  // asin argument guarded on the SAME axis it uses (the 2022 code checked X for the Y ratio)
  acc_angle[X] = asin(constrain(ay / total, -1.0f, 1.0f)) * (180.0f / PI);  // roll
  acc_angle[Y] = asin(constrain(ax / total, -1.0f, 1.0f)) * (180.0f / PI);  // pitch
}

void calculateAngles() {
  calculateGyroAngles();
  calculateAccelerometerAngles();

  if (initialized) {
    // complementary filter: high-pass gyro + low-pass accelerometer (tau ~ 10 s)
    gyro_angle[X] = gyro_angle[X] * CF_ALPHA + acc_angle[X] * (1.0f - CF_ALPHA);
    gyro_angle[Y] = gyro_angle[Y] * CF_ALPHA + acc_angle[Y] * (1.0f - CF_ALPHA);
  } else {
    resetGyroAngles();
    initialized = true;
  }

  // first-order low-pass on the angles (tau = -T/ln(0.9) = 38 ms) and on the rates (fc = -ln(0.7)/(2 pi T) = 14 Hz)
  measures[ROLL]  = measures[ROLL]  * 0.9f + gyro_angle[X] * 0.1f;
  measures[PITCH] = measures[PITCH] * 0.9f + gyro_angle[Y] * 0.1f;
  measures[YAW]   = gyro_raw[Z] / SSF_GYRO;

  angular_motions[ROLL]  = 0.7f * angular_motions[ROLL]  + 0.3f * gyro_raw[X] / SSF_GYRO;
  angular_motions[PITCH] = 0.7f * angular_motions[PITCH] + 0.3f * gyro_raw[Y] / SSF_GYRO;
  angular_motions[YAW]   = 0.7f * angular_motions[YAW]   + 0.3f * gyro_raw[Z] / SSF_GYRO;
  // (the 2022 version wrote a raw byte to Serial here every loop; removed — it
  //  corrupted the serial link and cost time inside the 4 ms budget)
}

void resetGyroAngles() {
  gyro_angle[X] = acc_angle[X];
  gyro_angle[Y] = acc_angle[Y];
}

// ----------------------------------------------------------------------------
// Set-points and PID
// ----------------------------------------------------------------------------
float calculateSetPoint(float angle, int channel_pulse) {
  float level_adjust = angle * LEVEL_GAIN;   // outer (angle) loop, proportional
  float set_point = 0;
  if (channel_pulse > PULSE_MID + STICK_DEADBAND_US)      set_point = channel_pulse - (PULSE_MID + STICK_DEADBAND_US);
  else if (channel_pulse < PULSE_MID - STICK_DEADBAND_US) set_point = channel_pulse - (PULSE_MID - STICK_DEADBAND_US);
  set_point -= level_adjust;
  set_point /= STICK_DIV;                    // -> deg/s
  return set_point;
}

float calculateYawSetPoint(int yaw_pulse, int throttle_pulse) {
  float set_point = 0;
  if (throttle_pulse > 1050) {
    set_point = calculateSetPoint(0, yaw_pulse);   // no absolute heading on yaw: rate only
  }
  return set_point;
}

void calculateSetPoints() {
  pid_set_points[YAW]   = calculateYawSetPoint(pulse[mode_mapping[YAW]], pulse[mode_mapping[THROTTLE]]);
  pid_set_points[PITCH] = calculateSetPoint(measures[PITCH], pulse[mode_mapping[PITCH]]);
  pid_set_points[ROLL]  = calculateSetPoint(measures[ROLL],  pulse[mode_mapping[ROLL]]);
}

void calculateErrors() {
  for (uint8_t a = 0; a < 3; a++) {
    errors[a] = angular_motions[a] - pid_set_points[a];
    error_sum[a] += errors[a];
    error_sum[a] = minMax(error_sum[a], -PID_LIMIT_US / KI[a], PID_LIMIT_US / KI[a]);  // anti wind-up
    delta_err[a] = errors[a] - previous_error[a];
    previous_error[a] = errors[a];
  }
}

void pidController() {
  int throttle = pulse[mode_mapping[THROTTLE]];
  float pid[3] = {0, 0, 0};

  for (uint8_t i = 0; i < 4; i++) esc_pulse[i] = throttle;

  if (throttle >= THROTTLE_ACTIVE) {
    for (uint8_t a = 0; a < 3; a++) {
      // u = Kp*e + Ki*sum(e) + Kd*delta(e)    (discrete PID, Ts = 4 ms folded into the gains)
      pid[a] = errors[a] * KP[a] + error_sum[a] * KI[a] + delta_err[a] * KD[a];
      pid[a] = minMax(pid[a], -PID_LIMIT_US, PID_LIMIT_US);
    }
    if (throttle > 1800) throttle = 1800;    // keep head-room for attitude control

    // motor mixer ("X" frame)
    float m[4];
    m[0] = throttle - pid[ROLL] - pid[PITCH] + pid[YAW];   // front-left
    m[1] = throttle + pid[ROLL] - pid[PITCH] - pid[YAW];   // front-right
    m[2] = throttle - pid[ROLL] + pid[PITCH] - pid[YAW];   // rear-left
    m[3] = throttle + pid[ROLL] + pid[PITCH] + pid[YAW];   // rear-right
    for (uint8_t i = 0; i < 4; i++) esc_pulse[i] = (uint16_t)minMax(m[i], ESC_IDLE, PULSE_MAX);
  }
  for (uint8_t i = 0; i < 4; i++) esc_pulse[i] = (uint16_t)minMax(esc_pulse[i], ESC_IDLE, PULSE_MAX);
}

void resetPidController() {
  for (uint8_t a = 0; a < 3; a++) {
    errors[a] = 0; error_sum[a] = 0; previous_error[a] = 0; delta_err[a] = 0;
  }
}

void stopAll() {
  for (uint8_t i = 0; i < 4; i++) esc_pulse[i] = PULSE_MIN;
}

float minMax(float value, float min_value, float max_value) {
  if (value > max_value) return max_value;
  if (value < min_value) return min_value;
  return value;
}

// ----------------------------------------------------------------------------
// Battery
// ----------------------------------------------------------------------------
bool isBatteryConnected() {
#if BATTERY_SENSE_PRESENT
  // first-order low-pass, same dynamics as the thesis (0.92 / 0.08)
  battery_cv = battery_cv * 0.92f + (analogRead(A0) + BATT_OFFSET) * BATT_SCALE * 0.08f;
  return battery_cv < BATT_FULL_CV && battery_cv > BATT_MIN_CV;
#else
  return false;
#endif
}

void compensateBatteryDrop() {
  if (isBatteryConnected()) {
    // ESC output falls with the pack voltage; scale the pulse by (12.40 V - Vbat)/35 V
    float k = (BATT_FULL_CV - battery_cv) / 3500.0f;
    for (uint8_t i = 0; i < 4; i++) {
      esc_pulse[i] = minMax(esc_pulse[i] + esc_pulse[i] * k, ESC_IDLE, PULSE_MAX);
    }
  }
}

// ----------------------------------------------------------------------------
// Payload sensors + telemetry
// ----------------------------------------------------------------------------
void readPayload() {
#if GAS_SENSOR_PRESENT
  // read only once per telemetry period (analogRead costs ~112 us)
  if (telemetry_count == 0) gas_adc = analogRead(A3);
#endif
}

// Frame: $T,<status>,<fs>,<roll x10>,<pitch x10>,<yawrate x10>,<gas adc>,<cps>,<cpm>,<vbat cV>*<xor>\n
// Max ~52 bytes < 64-byte TX buffer, so Serial.print never blocks the loop.
void sendTelemetry() {
  if (++telemetry_count < TELEMETRY_DIVIDER) return;
  telemetry_count = 0;

  uint16_t cpm = 0, cps = 0;
#if GEIGER_SENSOR_PRESENT
  noInterrupts(); cpm = cpm_rolling; cps = cps_last; interrupts();
#endif
  char body[64];
  snprintf(body, sizeof(body), "T,%u,%u,%d,%d,%d,%u,%u,%u,%d",
           status, failsafe ? 1 : 0,
           (int)(measures[ROLL] * 10), (int)(measures[PITCH] * 10),
           (int)(angular_motions[YAW] * 10),
           gas_adc, cps, cpm, battery_cv);
  uint8_t cs = 0;
  for (char *p = body; *p; p++) cs ^= (uint8_t)*p;
  char frame[72];
  snprintf(frame, sizeof(frame), "$%s*%02X\n", body, cs);
  if ((size_t)Serial.availableForWrite() >= strlen(frame)) Serial.print(frame);
}

// ----------------------------------------------------------------------------
// ESC pulse generation (software PWM, 250 Hz refresh)
// ----------------------------------------------------------------------------
void applyMotorSpeed() {
  while ((now_us = micros()) - loop_timer < LOOP_PERIOD_US);   // wait for the 4 ms tick
  loop_timer = now_us;

  PORTD |= B11110000;                         // D4..D7 HIGH
  while (PORTD & B11110000) {                 // until every pulse has ended (<= 2 ms)
    difference = micros() - loop_timer;
    if (difference >= esc_pulse[0]) PORTD &= B11101111;   // D4 LOW
    if (difference >= esc_pulse[1]) PORTD &= B11011111;   // D5 LOW
    if (difference >= esc_pulse[2]) PORTD &= B10111111;   // D6 LOW
    if (difference >= esc_pulse[3]) PORTD &= B01111111;   // D7 LOW
  }
}

// ----------------------------------------------------------------------------
// Interrupts
// ----------------------------------------------------------------------------
// ONE handler for every pin of PCINT0 (D8..D12). Measures HIGH time of each
// command channel and counts falling edges of the Geiger pulse.
ISR(PCINT0_vect) {
  uint32_t t = micros();
  uint8_t pins = PINB;

  for (uint8_t ch = 0; ch < 4; ch++) {
    uint8_t mask = (1 << ch);
    if (pins & mask) {
      if (prev_state[ch] == 0) { prev_state[ch] = 1; rise_time[ch] = t; }
    } else if (prev_state[ch] == 1) {
      prev_state[ch] = 0;
      uint32_t w = t - rise_time[ch];
      if (w >= 900 && w <= 2100) {            // reject glitches
        pulse_len[ch] = (uint16_t)w;
        last_seen_ms[ch] = millis();
      }
    }
  }

#if GEIGER_SENSOR_PRESENT
  uint8_t g = (pins & B00010000) ? 1 : 0;
  if (geiger_prev == 1 && g == 0) geiger_ticks++;   // falling edge = one ionisation event
  geiger_prev = g;
#endif
}

#if GEIGER_SENSOR_PRESENT
// 1 Hz gate: store counts of the last second in a 60-slot ring -> rolling CPM
ISR(TIMER1_COMPA_vect) {
  uint8_t c = geiger_ticks > 255 ? 255 : geiger_ticks;
  geiger_ticks = 0;
  cpm_rolling -= cps_ring[cps_idx];
  cps_ring[cps_idx] = c;
  cpm_rolling += c;
  cps_last = c;
  cps_idx = (cps_idx + 1) % 60;
}
#endif
