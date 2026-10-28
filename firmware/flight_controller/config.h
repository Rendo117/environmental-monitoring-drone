/*
 * config.h — compile-time configuration of the flight controller
 * Environmental-monitoring quadcopter (C. M. Chirea, Univ. of Craiova, 2022; revised 2026)
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef CONFIG_H
#define CONFIG_H

// ---------------------------------------------------------------------------
// Optional payload sensors. Set to 0 to remove a sensor from the build
// (the original thesis used GAS_SENSOR_PRESENT / GEYGER_SENSOR_PRESENT).
// ---------------------------------------------------------------------------
#define GAS_SENSOR_PRESENT     1   // MQ-2 analog output on A3
#define GEIGER_SENSOR_PRESENT  1   // Geiger pulse output on D12 (PB4 / PCINT4), active LOW
#define BATTERY_SENSE_PRESENT  1   // 3S LiPo through divider on A0

// ---------------------------------------------------------------------------
// Control loop
// ---------------------------------------------------------------------------
#define LOOP_FREQ_HZ      250                       // control loop frequency  [Hz]
#define LOOP_PERIOD_US    (1000000UL / LOOP_FREQ_HZ) // 4000 us
#define SSF_GYRO          65.5f                     // LSB per deg/s at +/-500 deg/s (MPU-6050 datasheet)
#define SSF_ACC           4096.0f                   // LSB per g at +/-8 g (ACCEL_CONFIG = 0x10)

// IMU axis signs. Internal convention (required by the motor mixer):
//   roll  + = left wing up,  pitch + = nose up,  yaw rate + = nose turning right.
// Rates and angles now use the SAME sign (rate = d(angle)/dt). The 2022 code
// integrated -gyroY for the pitch angle but used +gyroY for the pitch rate, which
// makes the self-levelling (outer) loop unstable on pitch: d(angle)/dt = +5*angle.
// Defaults keep the 2022 inner rate loops and accelerometer/gyro pairing.
// Set them with the bench procedure in documentation/calibration.md.
#define GYRO_X_SIGN   (+1)   // roll rate
#define GYRO_Y_SIGN   (+1)   // pitch rate
#define GYRO_Z_SIGN   (+1)   // yaw rate
#define ACC_Y_SIGN    (+1)   // lateral specific force  -> roll angle
#define ACC_X_SIGN    (-1)   // longitudinal specific force -> pitch angle

// Complementary filter weight (gyro). Time constant tau = dt*a/(1-a) = 0.004*0.9996/0.0004 ~ 10 s
#define CF_ALPHA          0.9996f

// Angle mode: set-point [deg/s] = (stick_us - level_gain * angle_deg) / stick_div
// Max angle = 492 us / 15 = 32.8 deg ; outer-loop gain = 15/3 = 5 (deg/s)/deg
#define LEVEL_GAIN        15.0f
#define STICK_DIV         3.0f
#define STICK_DEADBAND_US 8

// PID gains (inner rate loop), order: YAW, PITCH, ROLL (as tuned in the thesis)
static const float KP[3] = {4.0f, 1.3f, 1.3f};
static const float KI[3] = {0.02f, 0.04f, 0.04f};
static const float KD[3] = {0.0f, 18.0f, 18.0f};
#define PID_LIMIT_US      400.0f                    // max correction per axis [us]

// ---------------------------------------------------------------------------
// ESC / receiver pulse limits [us]
// ---------------------------------------------------------------------------
#define PULSE_MIN         1000
#define PULSE_MID         1500
#define PULSE_MAX         2000
#define ESC_IDLE          1100      // motors spin slowly when armed
#define THROTTLE_ACTIVE   1012      // PID engaged above this throttle
#define ARM_LOW           1012
#define ARM_HIGH          1988
#define ARM_CENTER_TOL    50        // yaw is "centred" if |yaw-1500| < 50 us

// ---------------------------------------------------------------------------
// Fail-safe: if no valid pulse is seen for FAILSAFE_TIMEOUT_MS on any channel,
// attitude set-points go to level and throttle ramps down by 1 us every
// FAILSAFE_RAMP_DIV loops (10 -> 25 us/s), then the motors are disarmed.
// Simulation (simulation/quad_sim.py): from 30 m, 250 us/s hits the ground at
// ~20 m/s, 25 us/s at a few m/s. Without an altimeter this is open loop; add a
// barometer (BMP280) for a real controlled descent.
// ---------------------------------------------------------------------------
#define FAILSAFE_TIMEOUT_MS 250
#define FAILSAFE_RAMP_DIV   10

// ---------------------------------------------------------------------------
// Telemetry over UART to the Raspberry Pi (Arduino TX -> divider -> RPi RX!)
// ---------------------------------------------------------------------------
#define SERIAL_BAUD        115200
#define TELEMETRY_DIVIDER  25       // 250 Hz / 25 = 10 Hz telemetry

// Battery: Vbat[0.01 V] = (ADC + BATT_OFFSET) * BATT_SCALE  (divider + series diode, see hardware/wiring.md)
// (steady state of the thesis filter v = 0.92 v + 0.09853 (ADC+65)  ->  0.09853/0.08 = 1.2317)
#define BATT_SCALE         1.2317f
#define BATT_OFFSET        65
#define BATT_FULL_CV       1240     // 12.40 V
#define BATT_MIN_CV        800      // below this: "no battery" (USB powered)

#endif
