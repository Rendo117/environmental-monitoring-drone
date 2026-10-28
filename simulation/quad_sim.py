#!/usr/bin/env python3
"""
quad_sim.py — nonlinear 6-DOF simulation of the quadcopter with an exact replica of
the flight-controller firmware (250 Hz loop, complementary filter, cascaded
angle-P / rate-PID, mixer, ESC limits, fail-safe ramp).

Model: Newton–Euler equations of a rigid body with four rotors (Luukkonen 2011,
the reference [13] of the thesis), body frame x forward, y left, z up.

    m * xi_dd  = -m g e_z + R(eta) * [0, 0, T]^T - A * xi_d
    I * nu_d   = -nu x (I nu) - Gamma + tau
    eta_d      = W_eta^-1 * nu
    f_i = k w_i^2 ,  tau_Mi = b w_i^2 ,  T_m * w_i_d + w_i = w_cmd(u_i)

Run:  python3 quad_sim.py           -> figures in ./results and ../docs/assets/img
SPDX-License-Identifier: MIT
"""
from __future__ import annotations

import os
from dataclasses import dataclass, field

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = [os.path.join(HERE, "results"), os.path.join(HERE, "..", "docs", "assets", "img")]

# ----------------------------------------------------------------------------- style
INK, INK2, MUTED, GRID = "#0b0b0b", "#52514e", "#8a8984", "#e6e5e1"
C1, C2, C3, C4 = "#2a78d6", "#eb6834", "#1baf7a", "#6250d6"
plt.rcParams.update({
    "figure.dpi": 110, "savefig.dpi": 160, "font.size": 10, "axes.titlesize": 11,
    "axes.edgecolor": MUTED, "axes.labelcolor": INK2, "xtick.color": INK2, "ytick.color": INK2,
    "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.8, "axes.spines.top": False,
    "axes.spines.right": False, "lines.linewidth": 2.0, "legend.frameon": False,
    "figure.facecolor": "white", "axes.facecolor": "white", "text.color": INK,
})


def save(fig, name):
    for d in OUT:
        os.makedirs(d, exist_ok=True)
        fig.savefig(os.path.join(d, name), bbox_inches="tight")
    plt.close(fig)
    print("saved", name)


# ----------------------------------------------------------------------------- parameters
@dataclass
class Params:
    # F450-class frame as in the photo of the built drone (estimates, see documentation)
    m: float = 1.20            # kg   frame + 4 motors + RPi 4 + Uno + sensors + 3S 2200 mAh
    g: float = 9.81
    L: float = 0.225           # m    motor-to-centre distance
    Ixx: float = 0.0115        # kg m^2
    Iyy: float = 0.0115
    Izz: float = 0.0226
    k: float = 8.5e-6          # N/(rad/s)^2      thrust coefficient (10x4.5" prop)
    b: float = 1.4e-7          # N m/(rad/s)^2    drag-torque coefficient
    Im: float = 3.4e-5         # kg m^2           rotor + prop inertia
    w_max: float = 1000.0      # rad/s at 2000 us
    tau_m: float = 0.04        # s     motor/ESC time constant
    A: tuple = (0.25, 0.25, 0.25)  # kg/s  linear drag (Luukkonen)

    @property
    def a(self):               # moment arm of each rotor in the X configuration
        return self.L / np.sqrt(2)

    def hover(self):
        f_h = self.m * self.g / 4
        w_h = np.sqrt(f_h / self.k)
        u_h = 1000 + 1000 * w_h / self.w_max
        return f_h, w_h, u_h


# firmware constants (config.h)
KP = np.array([4.0, 1.3, 1.3])     # yaw, pitch, roll
KI = np.array([0.02, 0.04, 0.04])
KD = np.array([0.0, 18.0, 18.0])
FREQ, SSF_GYRO, SSF_ACC = 250, 65.5, 4096.0
YAW, PITCH, ROLL = 0, 1, 2


def rot(phi, th, psi):
    c, s = np.cos, np.sin
    return np.array([
        [c(psi) * c(th), c(psi) * s(th) * s(phi) - s(psi) * c(phi), c(psi) * s(th) * c(phi) + s(psi) * s(phi)],
        [s(psi) * c(th), s(psi) * s(th) * s(phi) + c(psi) * c(phi), s(psi) * s(th) * c(phi) - c(psi) * s(phi)],
        [-s(th), c(th) * s(phi), c(th) * c(phi)]])


def W_inv(phi, th):
    c, s, t = np.cos, np.sin, np.tan
    return np.array([[1, s(phi) * t(th), c(phi) * t(th)],
                     [0, c(phi), -s(phi)],
                     [0, s(phi) / c(th), c(phi) / c(th)]])


# ----------------------------------------------------------------------------- firmware replica
@dataclass
class Firmware:
    gyro_angle: np.ndarray = field(default_factory=lambda: np.zeros(2))
    acc_angle: np.ndarray = field(default_factory=lambda: np.zeros(2))
    measures: np.ndarray = field(default_factory=lambda: np.zeros(3))
    rates: np.ndarray = field(default_factory=lambda: np.zeros(3))
    esum: np.ndarray = field(default_factory=lambda: np.zeros(3))
    eprev: np.ndarray = field(default_factory=lambda: np.zeros(3))
    init: bool = False

    @staticmethod
    def set_point(angle, pulse):
        sp = 0.0
        if pulse > 1508:
            sp = pulse - 1508
        elif pulse < 1492:
            sp = pulse - 1492
        return (sp - angle * 15) / 3

    legacy_pitch: bool = False      # True = 2022 sign error (angle integrates -gyroY, rate uses +gyroY)

    def step(self, gyro_raw, acc_raw, pulses):
        """gyro_raw/acc_raw in LSB as read from the MPU (signs already as in config.h defaults);
        pulses: dict roll/pitch/yaw/throttle [us]."""
        gx, gy, gz = gyro_raw
        ax, ay, az = acc_raw
        ax = -ax                                     # ACC_X_SIGN = -1
        if self.legacy_pitch:                        # 2022: pitch angle from -gyroY, accel sign matched to it
            gy_angle, ax = -gy, -ax
        else:
            gy_angle = gy
        self.gyro_angle[0] += gx / (FREQ * SSF_GYRO)
        self.gyro_angle[1] += gy_angle / (FREQ * SSF_GYRO)
        s = np.sin(gz * np.pi / (FREQ * SSF_GYRO * 180))
        if self.legacy_pitch:
            self.gyro_angle[1] += self.gyro_angle[0] * s
            self.gyro_angle[0] -= self.gyro_angle[1] * s
        else:
            self.gyro_angle[1] -= self.gyro_angle[0] * s
            self.gyro_angle[0] += self.gyro_angle[1] * s
        tot = np.sqrt(ax * ax + ay * ay + az * az)
        self.acc_angle[0] = np.degrees(np.arcsin(np.clip(ay / tot, -1, 1)))
        self.acc_angle[1] = np.degrees(np.arcsin(np.clip(ax / tot, -1, 1)))
        if self.init:
            self.gyro_angle = self.gyro_angle * 0.9996 + self.acc_angle * 0.0004
        else:
            self.gyro_angle = self.acc_angle.copy()
            self.init = True
        self.measures[ROLL] = self.measures[ROLL] * 0.9 + self.gyro_angle[0] * 0.1
        self.measures[PITCH] = self.measures[PITCH] * 0.9 + self.gyro_angle[1] * 0.1
        self.rates[ROLL] = 0.7 * self.rates[ROLL] + 0.3 * gx / SSF_GYRO
        self.rates[PITCH] = 0.7 * self.rates[PITCH] + 0.3 * gy / SSF_GYRO
        self.rates[YAW] = 0.7 * self.rates[YAW] + 0.3 * gz / SSF_GYRO

        thr = pulses["throttle"]
        sp = np.zeros(3)
        sp[YAW] = self.set_point(0, pulses["yaw"]) if thr > 1050 else 0.0
        sp[PITCH] = self.set_point(self.measures[PITCH], pulses["pitch"])
        sp[ROLL] = self.set_point(self.measures[ROLL], pulses["roll"])
        e = self.rates - sp
        self.esum = np.clip(self.esum + e, -400 / KI, 400 / KI)
        de = e - self.eprev
        self.eprev = e
        esc = np.full(4, float(thr))
        if thr >= 1012:
            pid = np.clip(e * KP + self.esum * KI + de * KD, -400, 400)
            t = min(thr, 1800)
            esc = np.array([t - pid[ROLL] - pid[PITCH] + pid[YAW],
                            t + pid[ROLL] - pid[PITCH] - pid[YAW],
                            t - pid[ROLL] + pid[PITCH] - pid[YAW],
                            t + pid[ROLL] + pid[PITCH] + pid[YAW]])
        return np.clip(esc, 1100, 2000), sp


# ----------------------------------------------------------------------------- plant
class Quad:
    # rotor positions (x forward, y left): FL, FR, RL, RR  ; spin: +1 = CW seen from above
    def __init__(self, p: Params, z0=0.0):
        self.p = p
        a = p.a
        self.pos = np.array([[a, a], [a, -a], [-a, a], [-a, -a]])
        self.spin = np.array([+1, -1, -1, +1])
        _, w_h, _ = p.hover()
        self.w = np.full(4, w_h)
        self.xi = np.array([0.0, 0.0, z0])
        self.v = np.zeros(3)
        self.eta = np.zeros(3)
        self.nu = np.zeros(3)
        self.I = np.diag([p.Ixx, p.Iyy, p.Izz])

    def step(self, u, dt, tau_ext=np.zeros(3)):
        p = self.p
        w_cmd = p.w_max * np.clip((u - 1000) / 1000, 0, 1)
        self.w += (w_cmd - self.w) * dt / p.tau_m
        f = p.k * self.w ** 2
        T = f.sum()
        tau = np.array([
            np.sum(self.pos[:, 1] * f),                 # roll   = sum(y_i f_i)
            -np.sum(self.pos[:, 0] * f),                # pitch  = -sum(x_i f_i)
            np.sum(self.spin * p.b * self.w ** 2),      # yaw: CW rotor -> +z reaction torque
        ]) + tau_ext
        w_gamma = np.sum(-self.spin * self.w)           # net rotor angular velocity along +z
        gyro = p.Im * np.cross(self.nu, np.array([0, 0, 1.0])) * w_gamma
        nu_d = np.linalg.solve(self.I, tau - np.cross(self.nu, self.I @ self.nu) - gyro)
        R = rot(*self.eta)
        acc = np.array([0, 0, -p.g]) + R @ np.array([0, 0, T]) / p.m - np.array(p.A) * self.v / p.m
        self.v += acc * dt
        self.xi += self.v * dt
        self.nu += nu_d * dt
        self.eta += W_inv(self.eta[0], self.eta[1]) @ self.nu * dt
        return acc

    def imu(self, acc_world, rng, noise=True):
        """MPU-6050 readings [LSB] for a sensor mounted so that, with the config.h
        default signs, the firmware sees: roll rate + = left wing up (= p),
        pitch rate + = nose up (= -q), yaw rate + = nose right (= -r)."""
        p_, q_, r_ = np.degrees(self.nu)
        gyro = np.array([p_, -q_, -r_]) * SSF_GYRO
        f_b = rot(*self.eta).T @ (acc_world + np.array([0, 0, self.p.g]))   # specific force, body
        acc = np.array([-f_b[0], f_b[1], f_b[2]]) / self.p.g * SSF_ACC      # raw X is negated by ACC_X_SIGN
        if noise:
            gyro += rng.normal(0, 0.05 * SSF_GYRO, 3)                 # 0.05 deg/s rms
            acc += rng.normal(0, 0.02 * SSF_ACC, 3)                   # 20 mg rms (vibration)
        return np.round(gyro), np.round(acc)


def run(seconds, stick_fn, p=Params(), z0=10.0, dist_fn=None, seed=1, failsafe=None, legacy_pitch=False, fs_div=1, baro=False):
    rng = np.random.default_rng(seed)
    q, fw = Quad(p, z0), Firmware(legacy_pitch=legacy_pitch)
    dt, sub = 0.0005, 8                         # plant at 2 kHz, controller every 4 ms
    n = int(seconds / dt)
    log = {k: [] for k in ["t", "phi", "theta", "psi_rate", "z", "vz", "sp_roll", "esc", "u_thr"]}
    esc = np.full(4, p.hover()[2])
    acc_w = np.zeros(3)
    stopped = False
    fs_thr = None
    for i in range(n):
        t = i * dt
        if i % sub == 0:
            sticks = stick_fn(t)
            if failsafe is not None and t >= failsafe:
                if baro:   # proposed: vertical-speed PI loop on a barometric/accel estimate, target -1 m/s
                    vz_m = q.v[2] + rng.normal(0, 0.15)
                    fw_vz_i = getattr(fw, "vz_i", 0.0) + (-1.0 - vz_m) * 0.004
                    fw.vz_i = fw_vz_i
                    fs_thr = int(round(p.hover()[2] + 40 * (-1.0 - vz_m) + 20 * fw_vz_i))
                    if q.xi[2] < 0.05:
                        fs_thr = 1100
                else:
                    fs_thr = sticks["throttle"] if fs_thr is None else fs_thr - (1 if (i // sub) % fs_div == 0 else 0)
                sticks = {"roll": 1500, "pitch": 1500, "yaw": 1500, "throttle": fs_thr}
                if fs_thr <= 1100:
                    stopped = True
            # first sample ~ average of the 2000 calibration samples (noise / sqrt(2000))
            g_raw, a_raw = q.imu(acc_w, rng, noise=(i > 0))
            esc_new, sp = fw.step(g_raw, a_raw, sticks)
            esc = np.full(4, 1000.0) if stopped else esc_new
            log["t"].append(t)
            log["phi"].append(np.degrees(q.eta[0]))
            log["theta"].append(-np.degrees(q.eta[1]))
            log["psi_rate"].append(-np.degrees(q.nu[2]))
            log["z"].append(q.xi[2])
            log["vz"].append(q.v[2])
            log["sp_roll"].append(sp[ROLL])
            log["esc"].append(esc.copy())
            log["u_thr"].append(sticks["throttle"])
        tau_ext = dist_fn(t) if dist_fn else np.zeros(3)
        acc_w = q.step(esc, dt, tau_ext)
        if q.xi[2] <= 0 and failsafe is not None:
            break
        if abs(q.eta[0]) > 1.2 or abs(q.eta[1]) > 1.2:     # > ~70 deg: crashed / unrecoverable
            break
    return {k: np.array(v) for k, v in log.items()}


# ----------------------------------------------------------------------------- experiments
def exp_roll_step(p):
    u_h = round(p.hover()[2])

    def sticks(t):
        roll = 1600 if 1.0 <= t < 3.5 else 1500
        return {"roll": roll, "pitch": 1500, "yaw": 1500, "throttle": u_h}

    r = run(5.0, sticks, p)
    target = (1600 - 1508) / 15
    fig, ax = plt.subplots(2, 1, figsize=(8, 6), sharex=True, gridspec_kw={"height_ratios": [3, 2]})
    ax[0].plot(r["t"], r["phi"], color=C1, label="roll angle φ (simulated)")
    ax[0].step([0, 1, 3.5, 5], [0, target, 0, 0], where="post", color=MUTED, lw=1.2, ls="--",
               label=f"steady-state target (1600−1508)/15 = {target:.2f}°")
    ax[0].set_ylabel("angle [°]")
    ax[0].set_title("Roll-stick step 1500 → 1600 µs: closed-loop response with the thesis PID gains")
    ax[0].legend(loc="lower center")
    esc = np.vstack(r["esc"])
    ax[1].plot(r["t"], esc[:, 0], color=C1, lw=1.4, label="ESC1 front-left")
    ax[1].plot(r["t"], esc[:, 1], color=C2, lw=1.4, label="ESC2 front-right")
    ax[1].set_ylabel("pulse [µs]")
    ax[1].set_xlabel("time [s]")
    ax[1].legend(loc="lower center", ncol=2)
    save(fig, "sim-roll-step.png")
    i = (r["t"] > 1.0) & (r["t"] < 3.5)
    phi = r["phi"][i]
    t = r["t"][i] - 1.0
    ss = phi[-50:].mean()
    rise = t[np.argmax(phi >= 0.9 * ss)] - t[np.argmax(phi >= 0.1 * ss)]
    over = (phi.max() - ss) / ss * 100
    settle = t[np.where(np.abs(phi - ss) > 0.05 * abs(ss))[0][-1]]
    return {"target_deg": target, "steady_deg": ss, "rise_s": rise, "overshoot_pct": over, "settle_5pct_s": settle}


def exp_disturbance(p):
    u_h = round(p.hover()[2])
    sticks = lambda t: {"roll": 1500, "pitch": 1500, "yaw": 1500, "throttle": u_h}
    gust = lambda t: np.array([0.4, 0.0, 0.0]) if 1.0 <= t < 1.05 else np.zeros(3)
    r = run(3.0, sticks, p, dist_fn=gust)
    fig, ax = plt.subplots(figsize=(8, 3.6))
    ax.plot(r["t"], r["phi"], color=C2, label="roll angle φ")
    ax.axvspan(1.0, 1.05, color=GRID)
    ax.annotate("0.4 N·m roll gust, 50 ms", (1.05, r["phi"].max()), xytext=(1.4, r["phi"].max() * 0.9),
                color=INK2, arrowprops=dict(arrowstyle="-", color=MUTED))
    ax.set_xlabel("time [s]")
    ax.set_ylabel("angle [°]")
    ax.set_title("Disturbance rejection: impulse torque on the roll axis while hovering")
    save(fig, "sim-disturbance.png")
    return {"peak_deg": float(r["phi"].max()),
            "recover_1deg_s": float(r["t"][np.where(np.abs(r["phi"]) > 1)[0][-1]] - 1.0)}


def exp_failsafe(p):
    _, _, u_h = p.hover()
    sticks = lambda t: {"roll": 1500, "pitch": 1500, "yaw": 1500, "throttle": round(u_h)}
    fast = run(60.0, sticks, p, z0=30.0, failsafe=1.0, fs_div=1)     # 2022-style: 1 us per loop
    slow = run(60.0, sticks, p, z0=30.0, failsafe=1.0, fs_div=10)    # revised default: 25 us/s
    baro = run(60.0, sticks, p, z0=30.0, failsafe=1.0, baro=True)    # proposed hardware upgrade
    fig, ax = plt.subplots(2, 1, figsize=(8, 5.6), sharex=True)
    ax[0].plot(fast["t"], fast["z"], color=C2, label="ramp 250 µs/s")
    ax[0].plot(slow["t"], slow["z"], color=C1, label="ramp 25 µs/s (new default)")
    ax[0].plot(baro["t"], baro["z"], color=C3, label="proposed: barometer + vertical-speed loop (−1 m/s)")
    ax[0].set_ylabel("altitude z [m]")
    ax[0].set_title("Fail-safe descent from 30 m hover (link lost at t = 1 s)")
    ax[0].legend(loc="upper right")
    ax[1].plot(fast["t"], fast["u_thr"], color=C2)
    ax[1].plot(slow["t"], slow["u_thr"], color=C1)
    ax[1].plot(baro["t"], baro["u_thr"], color=C3, lw=1.2)
    ax[1].axhline(u_h, color=MUTED, ls="--", lw=1.2)
    ax[1].text(slow["t"][-1], u_h + 8, f"hover ≈ {u_h:.0f} µs", ha="right", color=INK2)
    ax[1].set_ylabel("throttle [µs]")
    ax[1].set_xlabel("time [s]")
    save(fig, "sim-failsafe.png")
    return {"fast_impact_mps": float(-fast["vz"][-1]), "fast_time_s": float(fast["t"][-1] - 1.0),
            "slow_impact_mps": float(-slow["vz"][-1]), "slow_time_s": float(slow["t"][-1] - 1.0),
            "baro_impact_mps": float(-baro["vz"][-1]), "baro_time_s": float(baro["t"][-1] - 1.0)}


def exp_complementary(p):
    """Open-loop test of the attitude estimator: slow swinging + vibration + gyro bias."""
    rng = np.random.default_rng(3)
    T, dt = 60.0, 1 / FREQ
    t = np.arange(0, T, dt)
    true = 10 * np.sin(2 * np.pi * 0.1 * t) * (t > 5)
    rate = np.gradient(true, dt)
    bias = 0.05                                           # deg/s residual after calibration
    gyro = (rate + bias + rng.normal(0, 0.05, t.size)) * SSF_GYRO
    acc_ang = true + rng.normal(0, 2.0, t.size)           # 2 deg rms vibration noise
    g_only = np.cumsum(gyro / (FREQ * SSF_GYRO))
    fused = np.zeros_like(t)
    x = acc_ang[0]
    for i in range(t.size):
        x = (x + gyro[i] / (FREQ * SSF_GYRO)) * 0.9996 + acc_ang[i] * 0.0004
        fused[i] = x
    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(t, acc_ang, color=GRID, lw=0.8, label="accelerometer only (noisy)")
    ax.plot(t, g_only, color=C2, label="gyroscope only (drifts with bias)")
    ax.plot(t, fused, color=C1, label="complementary filter α = 0.9996 (τ ≈ 10 s)")
    ax.plot(t, true, color=INK, lw=1.0, ls="--", label="true angle")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("angle [°]")
    ax.set_title("Attitude estimation: why the gyro and accelerometer are fused")
    ax.legend(loc="upper left", fontsize=9)
    save(fig, "sim-complementary.png")
    err = fused[int(10 / dt):] - true[int(10 / dt):]
    return {"rms_fused_deg": float(np.sqrt(np.mean(err ** 2))),
            "rms_acc_deg": 2.0, "gyro_drift_60s_deg": float(g_only[-1] - true[-1])}


def exp_pitch_sign_bug(p):
    """Same pitch-stick step with the 2022 sign convention and with the corrected one."""
    u_h = round(p.hover()[2])

    def sticks(t):
        return {"roll": 1500, "pitch": 1580 if 1.0 <= t < 3.0 else 1500, "yaw": 1500, "throttle": u_h}

    new = run(4.0, sticks, p)
    old = run(4.0, sticks, p, legacy_pitch=True)
    fig, ax = plt.subplots(figsize=(8, 3.8))
    ax.plot(new["t"], new["theta"], color=C1, label="corrected firmware (rate = d(angle)/dt)")
    ax.plot(old["t"], old["theta"], color=C2, label="2022 firmware (angle from −gyroY, rate from +gyroY)")
    ax.axhline((1580 - 1508) / 15, color=MUTED, ls="--", lw=1.2)
    ax.set_ylim(-10, 75)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("pitch angle, nose up [°]")
    ax.set_title("Pitch self-levelling: the 2022 sign mismatch makes the angle loop diverge")
    ax.legend(loc="upper left", fontsize=9)
    save(fig, "sim-pitch-sign-bug.png")
    return {"corrected_final_deg": float(new["theta"][-1]),
            "legacy_time_to_70deg_s": float(old["t"][-1] - 1.0) if old["t"][-1] < 3.99 else None}


def exp_bode(p):
    """Inner roll-rate loop, linearised at hover (units: deg/s and us)."""
    _, w_h, _ = p.hover()
    df_du = 2 * p.k * w_h * p.w_max / 1000           # N per us of ESC pulse
    dtau_du = 4 * p.a * df_du                        # roll torque per us of PID output
    K = np.degrees(dtau_du / p.Ixx)                  # (deg/s^2) per us
    Ts = 1 / FREQ
    w = np.logspace(-0.5, np.log10(np.pi / Ts * 0.999), 800)
    s = 1j * w
    z = np.exp(s * Ts)
    C = KP[ROLL] + KI[ROLL] / (1 - 1 / z) + KD[ROLL] * (1 - 1 / z)
    F = 0.3 / (1 - 0.7 / z)                          # rate low-pass in the firmware
    P = K / (s * (p.tau_m * s + 1))
    ZOH = (1 - np.exp(-s * Ts)) / (s * Ts)
    L = C * F * P * ZOH * np.exp(-s * Ts)            # + one sample computation delay
    mag = 20 * np.log10(np.abs(L))
    ph = np.degrees(np.unwrap(np.angle(L)))
    ic = np.where(np.diff(np.sign(mag)))[0][0]
    wc, pm = w[ic], 180 + ph[ic]
    ip = np.where(ph < -180)[0]
    gm = -mag[ip[0]] if ip.size else np.inf
    wp = w[ip[0]] if ip.size else np.nan
    fig, ax = plt.subplots(2, 1, figsize=(8, 5.6), sharex=True)
    ax[0].semilogx(w / (2 * np.pi), mag, color=C1)
    ax[0].axhline(0, color=MUTED, lw=1)
    ax[0].set_ylabel("|L(jω)| [dB]")
    ax[0].set_title(f"Open-loop Bode plot, roll-rate loop at hover — phase margin {pm:.0f}°, gain margin {gm:.1f} dB")
    ax[1].semilogx(w / (2 * np.pi), ph, color=C2)
    ax[1].axhline(-180, color=MUTED, lw=1)
    ax[1].axvline(wc / (2 * np.pi), color=MUTED, ls=":", lw=1)
    ax[1].text(wc / (2 * np.pi) * 1.08, -100, f"crossover {wc / (2 * np.pi):.1f} Hz", color=INK2)
    ax[1].set_ylabel("∠L(jω) [°]")
    ax[1].set_xlabel("frequency [Hz]")
    save(fig, "sim-bode.png")
    return {"K_deg_s2_per_us": K, "dtau_du_Nm_per_us": dtau_du, "crossover_hz": wc / (2 * np.pi),
            "phase_margin_deg": pm, "gain_margin_db": gm, "phase_crossover_hz": wp / (2 * np.pi)}


def exp_geiger():
    T = np.linspace(5, 600, 400)
    fig, ax = plt.subplots(figsize=(8, 3.8))
    for cpm, col, lab in [(20, C1, "20 CPM (natural background)"), (200, C2, "200 CPM"),
                          (2000, C3, "2000 CPM")]:
        N = cpm * T / 60
        ax.plot(T, 100 / np.sqrt(N), color=col, label=lab)
    ax.axvline(60, color=MUTED, ls="--", lw=1)
    ax.text(64, 60, "firmware window 60 s", color=INK2)
    ax.set_ylim(0, 100)
    ax.set_xlabel("counting time [s]")
    ax.set_ylabel("relative uncertainty 1σ [%]")
    ax.set_title("Poisson counting statistics: σ/N = 1/√N")
    ax.legend()
    save(fig, "sim-geiger-uncertainty.png")


def exp_mq2():
    ppm = np.logspace(np.log10(200), 4, 100)
    fig, ax = plt.subplots(figsize=(6.5, 4.2))
    for gas, (x0, y0, m), col in [("LPG", (2.3, 0.21, -0.47), C1), ("CO", (2.3, 0.72, -0.34), C2),
                                  ("smoke", (2.3, 0.53, -0.44), C3)]:
        ax.loglog(ppm, 10 ** (m * (np.log10(ppm) - x0) + y0), color=col, label=gas)
    ax.axhline(9.83, color=MUTED, ls="--", lw=1)
    ax.text(250, 7.6, "clean air  Rs/R0 = 9.83", color=INK2)
    ax.set_xlabel("concentration [ppm]")
    ax.set_ylabel("Rs / R0")
    ax.set_title("MQ-2 sensitivity curves (datasheet fit)")
    ax.legend()
    save(fig, "sim-mq2-curves.png")


def main():
    p = Params()
    f_h, w_h, u_h = p.hover()
    print(f"hover: f_i = {f_h:.3f} N, w_h = {w_h:.0f} rad/s ({w_h * 60 / 2 / np.pi:.0f} rpm), u_h = {u_h:.0f} us")
    res = {"hover": {"f_N": f_h, "w_rad_s": w_h, "u_us": u_h}}
    res["roll_step"] = exp_roll_step(p)
    res["disturbance"] = exp_disturbance(p)
    res["failsafe"] = exp_failsafe(p)
    res["complementary"] = exp_complementary(p)
    res["bode"] = exp_bode(p)
    res["pitch_sign_bug"] = exp_pitch_sign_bug(p)
    exp_geiger()
    exp_mq2()
    import json
    with open(os.path.join(HERE, "results", "summary.json"), "w") as fh:
        json.dump(res, fh, indent=2, default=float)
    print(json.dumps(res, indent=2, default=float))


if __name__ == "__main__":
    main()
