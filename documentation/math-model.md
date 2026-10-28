# Mathematical model of the quadcopter, extended and corrected

This chapter replaces and extends Chapter 3 of the 2022 thesis (*Modelul matematic al dronei*).
It follows Luukkonen, *Modelling and control of quadcopter* (Aalto University, 2011), which
is reference [13] of the thesis, and adds the parts the original chapter did not cover: the
translational dynamics, hover equilibrium, X-frame allocation, linearisation, the controller
actually running on the ATmega328P, the estimator, and the sensor models. Each result is
checked numerically in [`simulation/quad_sim.py`](../simulation/quad_sim.py).

Corrections to the 2022 text are marked **⚠ correction**.

---

## 1. Generalised coordinates

The absolute position of the centre of mass in the inertial frame $\{x_I, y_I, z_I\}$ ($z$ up) is
$\boldsymbol\xi$. The attitude is given by the Z-Y-X Euler angles $\boldsymbol\eta$: roll $\phi$
about $x$, pitch $\theta$ about $y$, yaw $\psi$ about $z$.

$$
\boldsymbol\xi=\begin{bmatrix}x\\y\\z\end{bmatrix},\qquad
\boldsymbol\eta=\begin{bmatrix}\phi\\\theta\\\psi\end{bmatrix},\qquad
\mathbf q=\begin{bmatrix}\boldsymbol\xi\\\boldsymbol\eta\end{bmatrix}\in\mathbb R^{6}
$$

**⚠ correction:** the thesis wrote $\mathbf q = [-\boldsymbol\xi;\ \boldsymbol\eta]$. The minus sign is a typo.

Body-frame linear and angular velocities:

$$
\mathbf V_B=\begin{bmatrix}v_{x,B}\\v_{y,B}\\v_{z,B}\end{bmatrix},\qquad
\boldsymbol\nu=\begin{bmatrix}p\\q\\r\end{bmatrix}
$$

## 2. Kinematics

**Rotation matrix** (body → inertial), with $S_x=\sin x$, $C_x=\cos x$:

$$
R = R_z(\psi)R_y(\theta)R_x(\phi)=
\begin{bmatrix}
C_\psi C_\theta & C_\psi S_\theta S_\phi - S_\psi C_\phi & C_\psi S_\theta C_\phi + S_\psi S_\phi\\
S_\psi C_\theta & S_\psi S_\theta S_\phi + C_\psi C_\phi & S_\psi S_\theta C_\phi - C_\psi S_\phi\\
-S_\theta & C_\theta S_\phi & C_\theta C_\phi
\end{bmatrix}
$$

$R\in SO(3)$: $R^{-1}=R^{\mathsf T}$ and $\det R = 1$.

**Euler-rate transformation:**

$$
\dot{\boldsymbol\eta}=W_\eta^{-1}\boldsymbol\nu,\qquad
W_\eta^{-1}=\begin{bmatrix}1 & S_\phi T_\theta & C_\phi T_\theta\\ 0 & C_\phi & -S_\phi\\ 0 & S_\phi/C_\theta & C_\phi/C_\theta\end{bmatrix},\qquad
W_\eta=\begin{bmatrix}1&0&-S_\theta\\0&C_\phi&C_\theta S_\phi\\0&-S_\phi&C_\theta C_\phi\end{bmatrix}
$$

Since $\det W_\eta = \cos\theta$, the matrix is invertible if and only if

$$
\theta \neq \frac{(2k-1)\pi}{2},\qquad k\in\mathbb Z .
$$

**⚠ correction:** the thesis wrote $\theta \neq (2k-1)\phi/2$. The singularity (gimbal lock) is at
$\theta = \pm 90^\circ$ and does not depend on $\phi$.

## 3. Rotor forces and torques

Rotor $i$, spinning at $\omega_i$, produces a thrust along its axis and a reaction (drag) torque:

$$
f_i = k\,\omega_i^{2},\qquad \tau_{M_i} = b\,\omega_i^{2} + I_M\,\dot\omega_i
$$

where $k$ is the lift constant, $b$ the drag constant and $I_M$ the rotor inertia. The $I_M\dot\omega_i$
term is small and usually neglected. Total thrust in the body frame:

$$
T=\sum_{i=1}^{4} f_i = k\sum_{i=1}^{4}\omega_i^{2},\qquad \mathbf T_B=\begin{bmatrix}0\\0\\T\end{bmatrix}
$$

### 3.1 "+" configuration (the one written in the thesis)

$$
\boldsymbol\tau_B=\begin{bmatrix}\tau_\phi\\\tau_\theta\\\tau_\psi\end{bmatrix}=
\begin{bmatrix} l k\,(-\omega_2^2+\omega_4^2)\\ l k\,(-\omega_1^2+\omega_3^2)\\ \sum_{i=1}^4 (-1)^{i+1}\,\tau_{M_i}\end{bmatrix}
$$

**⚠ correction:** the yaw torque needs the alternating sign $(-1)^{i+1}$, or an equivalent
signed sum. Adjacent rotors spin in opposite directions, so their reaction torques cancel. A plain
$\sum\tau_{M_i}$ would never be zero, and the aircraft could not hold its heading.

### 3.2 "X" configuration (the built drone)

The firmware mixer and the photo of the aircraft show an **X** frame. Motors sit at
$(\pm a, \pm a)$ with $a = l/\sqrt2$, numbered M1 front-left, M2 front-right, M3 rear-left and
M4 rear-right. M1 and M4 spin CW, M2 and M3 CCW. With $\mathbf r_i\times f_i\mathbf e_z = (y_i f_i,\,-x_i f_i,\,0)$:

$$
\begin{bmatrix}T\\\tau_\phi\\\tau_\theta\\\tau_\psi\end{bmatrix}=
\underbrace{\begin{bmatrix}
k&k&k&k\\
ak&-ak&ak&-ak\\
-ak&-ak&ak&ak\\
b&-b&-b&b
\end{bmatrix}}_{M}
\begin{bmatrix}\omega_1^2\\\omega_2^2\\\omega_3^2\\\omega_4^2\end{bmatrix},
\qquad
M^{-1}=\frac14\begin{bmatrix}
1/k & 1/(ak) & -1/(ak) & 1/b\\
1/k & -1/(ak) & -1/(ak) & -1/b\\
1/k & 1/(ak) & 1/(ak) & -1/b\\
1/k & -1/(ak) & 1/(ak) & 1/b
\end{bmatrix}
$$

$M^{-1}$ is the **control-allocation** matrix. Up to sign conventions and the linear
µs→ω² approximation, it is the motor mixer in the firmware:

$$
\begin{aligned}
u_1 &= T - r - p + y, & u_2 &= T + r - p - y,\\
u_3 &= T - r + p - y, & u_4 &= T + r + p + y,
\end{aligned}
$$

Here $r, p, y$ are the PID outputs (µs) for roll, pitch and yaw, with the firmware convention
roll + = left side up, pitch + = nose up, yaw + = nose right.

## 4. Dynamics (Newton–Euler)

**Translation** (inertial frame), with the linear drag matrix $A=\mathrm{diag}(A_x,A_y,A_z)$:

$$
m\ddot{\boldsymbol\xi}= \begin{bmatrix}0\\0\\-mg\end{bmatrix} + R\,\mathbf T_B - A\dot{\boldsymbol\xi}
\quad\Longleftrightarrow\quad
\begin{cases}
\ddot x = \dfrac{T}{m}\,(C_\psi S_\theta C_\phi + S_\psi S_\phi) - \dfrac{A_x}{m}\dot x\\[6pt]
\ddot y = \dfrac{T}{m}\,(S_\psi S_\theta C_\phi - C_\psi S_\phi) - \dfrac{A_y}{m}\dot y\\[6pt]
\ddot z = -g + \dfrac{T}{m}\,C_\theta C_\phi - \dfrac{A_z}{m}\dot z
\end{cases}
$$

**Rotation** (body frame, Euler's equation with rotor gyroscopic moment $\boldsymbol\Gamma$):

$$
I\dot{\boldsymbol\nu} + \boldsymbol\nu\times(I\boldsymbol\nu) + \boldsymbol\Gamma = \boldsymbol\tau_B,\qquad
\boldsymbol\Gamma = I_M\,(\boldsymbol\nu\times\mathbf e_z)\,\omega_\Gamma,\qquad
\omega_\Gamma = -\omega_1+\omega_2+\omega_3-\omega_4
$$

With $I=\mathrm{diag}(I_{xx},I_{yy},I_{zz})$:

$$
\begin{aligned}
\dot p &= \frac{I_{yy}-I_{zz}}{I_{xx}}\,q r - \frac{I_M}{I_{xx}}\,q\,\omega_\Gamma + \frac{\tau_\phi}{I_{xx}}\\
\dot q &= \frac{I_{zz}-I_{xx}}{I_{yy}}\,p r + \frac{I_M}{I_{yy}}\,p\,\omega_\Gamma + \frac{\tau_\theta}{I_{yy}}\\
\dot r &= \frac{I_{xx}-I_{yy}}{I_{zz}}\,p q + \frac{\tau_\psi}{I_{zz}}
\end{aligned}
$$

The system has 6 degrees of freedom and only 4 independent inputs ($T,\tau_\phi,\tau_\theta,\tau_\psi$),
so it is **under-actuated**. Horizontal motion is obtained only by tilting the thrust vector.

**⚠ correction:** the thesis says "we can control only φ, θ and ψ". The inputs are actually the
total thrust **and** the three torques. Altitude $z$ is directly actuated, while $x$ and $y$ are not.

## 5. Hover equilibrium (numerical values used in the simulation)

| Quantity | Symbol | Value | Basis |
|---|---|---|---|
| mass | $m$ | 1.20 kg | F450 frame, 4 motors, RPi 4, Uno, sensors, 3S 2200 mAh |
| arm length | $l$ | 0.225 m | F450 |
| inertia | $I_{xx}=I_{yy}$, $I_{zz}$ | 0.0115, 0.0226 kg·m² | point-mass estimate |
| lift constant | $k$ | 8.5·10⁻⁶ N·s² | 10×4.5″ propeller, ~8.5 N at 1000 rad/s |
| drag constant | $b$ | 1.4·10⁻⁷ N·m·s² | $b/k \approx 0.016$ m |
| motor time constant | $\tau_m$ | 40 ms | typical 920–1000 KV + 30 A ESC |

At hover $\ddot{\boldsymbol\xi}=0$ and $\phi=\theta=0$:

$$
4k\omega_h^2 = mg\ \Rightarrow\ \omega_h=\sqrt{\frac{mg}{4k}} = 588\ \text{rad/s}\ (5\,620\ \text{rpm}),\qquad
f_h = 2.94\ \text{N per rotor},\qquad u_h \approx 1588\ \mu\text{s}
$$

> These parameters are **estimates** for an F450-class frame. Measure $m$, weigh the thrust on a
> scale, and do a bifilar-pendulum test for $I$ to calibrate the model to the real aircraft.

## 6. Linearisation about hover

For small angles ($S_x\approx x$, $C_x\approx1$, $\psi=0$) and $T\approx mg$:

$$
\ddot x \approx g\,\theta,\qquad \ddot y \approx -g\,\phi,\qquad \ddot z \approx \frac{\Delta T}{m},\qquad
\ddot\phi \approx \frac{\tau_\phi}{I_{xx}},\quad \ddot\theta\approx\frac{\tau_\theta}{I_{yy}},\quad \ddot\psi\approx\frac{\tau_\psi}{I_{zz}}
$$

Assume a first-order motor, $\omega = \omega_{max}\,(u-1000)/1000$, and linearise $f=k\omega^2$ at $\omega_h$:

$$
\frac{\partial f}{\partial u} = 2k\omega_h\frac{\omega_{max}}{1000\,\mu\text{s}} = 0.0100\ \tfrac{\text{N}}{\mu\text{s}},\qquad
\frac{\partial\tau_\phi}{\partial u_{roll}} = 4a\,\frac{\partial f}{\partial u} = 6.37\cdot10^{-3}\ \tfrac{\text{N·m}}{\mu\text{s}}
$$

The roll-rate plant (input: PID output in µs, output: rate in °/s) is then

$$
G_\omega(s)=\frac{\Omega_\phi(s)}{U(s)}=\frac{K}{s\,(\tau_m s+1)},\qquad
K=\frac{180}{\pi}\cdot\frac{4a\,\partial f/\partial u}{I_{xx}} = 31.7\ \frac{^\circ/\text{s}^2}{\mu\text{s}}
$$

and the roll angle is $\Phi(s)=\Omega_\phi(s)/s$.

## 7. Controller implemented in the firmware

### 7.1 Inner loop: discrete PID on the angular rate (250 Hz, $T_s = 4$ ms)

$$
u_k = K_p\,e_k + K_i\sum_{j\le k} e_j + K_d\,(e_k-e_{k-1}),\qquad |u_k|\le 400\ \mu\text{s}
$$

| axis | $K_p$ | $K_i$ | $K_d$ | continuous equivalent $K_i/T_s$ | $K_d\,T_s$ |
|---|---|---|---|---|---|
| roll, pitch | 1.3 | 0.04 | 18 | 10 s⁻¹ | 0.072 s |
| yaw | 4.0 | 0.02 | 0 | 5 s⁻¹ | — |

Anti-windup clamps the integrator so its contribution stays within $\pm400$ µs: $\left|\sum e\right| \le 400/K_i$.

### 7.2 Outer loop: self-levelling (angle mode)

$$
\omega^{*} = \frac{u_{stick} - 1500 \mp 8 - 15\,\hat\varphi}{3}
\quad\Rightarrow\quad K_{angle}=\frac{15}{3}=5\ \text{s}^{-1},\qquad
\varphi_{max}=\frac{2000-1508}{15}=32.8^\circ
$$

At steady state $\omega^*=0$, so the attitude settles at $\hat\varphi=(u_{stick}-1508)/15$.
A 1600 µs roll stick commands 6.13°.

**⚠ correction (firmware bug):** in the 2022 code, the pitch **angle** integrated $-g_y$ while the
pitch **rate** fed to the PID used $+g_y$. With rate $=-\dot\varphi$ the outer loop gives
$\dot\varphi = +5\varphi$, an unstable pole at $s=+5$ s⁻¹. The angle grows by a factor of e
every 0.2 s. Simulated, the aircraft flips past 70° about 0.6 s after a small pitch command
([figure](../docs/assets/img/sim-pitch-sign-bug.png)). The revised firmware applies explicit
`GYRO_*_SIGN`/`ACC_*_SIGN` constants so that rate $=\dot\varphi$ on every axis.

### 7.3 Stability margins (roll-rate loop, linearised)

The open loop is
$L(j\omega)=C(e^{j\omega T})\,F(e^{j\omega T})\,G_\omega(j\omega)\,\frac{1-e^{-j\omega T}}{j\omega T}\,e^{-j\omega T}$.
It includes the PID, the firmware rate low-pass $F(z)=0.3/(1-0.7z^{-1})$, the zero-order hold and
one sample of computation delay. Results from `quad_sim.py`:

| crossover frequency | phase margin | gain margin | phase-crossover |
|---|---|---|---|
| 7.4 Hz | **52°** | **10.6 dB** | 18.7 Hz |

The thesis gains therefore give a well-damped inner loop for the assumed airframe.
In the nonlinear simulation a 1500→1600 µs roll step has a 10–90 % rise time of 0.86 s and <1 % overshoot.

## 8. Attitude estimation

### 8.1 Sensors
* gyroscope ±500 °/s → $S_g = 65.5$ LSB/(°/s); accelerometer ±8 g → $S_a = 4096$ LSB/g
  (**⚠ correction:** the comment on `ACCEL_CONFIG = 0x10` said "±500 °/s").
* accelerometer tilt: $\varphi_{acc}=\arcsin\left(a_y/\lVert\mathbf a\rVert\right)$, $\theta_{acc}=\arcsin\left(a_x/\lVert\mathbf a\rVert\right)$.

### 8.2 Complementary filter

$$
\hat\varphi_k=\alpha\left(\hat\varphi_{k-1}+\frac{g_k}{S_g}T_s\right)+(1-\alpha)\,\varphi_{acc,k},\qquad \alpha=0.9996
$$

In the continuous domain this is a high-pass on the integrated gyro plus a low-pass on the accelerometer:

$$
\hat\Phi(s)=\frac{\tau s}{\tau s+1}\cdot\frac{\Omega(s)}{s}+\frac{1}{\tau s+1}\,\Phi_{acc}(s),\qquad
\tau = -\frac{T_s}{\ln\alpha}\approx\frac{\alpha T_s}{1-\alpha}=10.0\ \text{s}
$$

A residual gyro bias $\beta$ gives a steady-state error $e_{ss}=\beta\,\tau$, so 0.05 °/s produces 0.5°.
On a multirotor the measured specific force stays almost aligned with the body $z$ axis in
steady flight. The accelerometer therefore observes tilt only weakly, which is why $\tau$ must be
long. It is also why the revised firmware initialises the angle from the **average** of the
2000 calibration samples instead of one noisy sample.

### 8.3 First-order IIR filters in the firmware

$y_k = a\,y_{k-1}+(1-a)\,x_k$ has time constant $\tau=-T_s/\ln a$ and cut-off $f_c=-\ln a/(2\pi T_s)$:

| filter | $a$ | $\tau$ | $f_c$ |
|---|---|---|---|
| angle smoothing | 0.9 | 38 ms | 4.2 Hz |
| rate smoothing | 0.7 | 11 ms | 14.2 Hz |
| battery voltage | 0.92 | 48 ms | 3.3 Hz |

## 9. Timing and signals

* **I²C clock** (prescaler 1): $f_{SCL}=\dfrac{f_{CPU}}{16+2\,\mathrm{TWBR}}=\dfrac{16\ \text{MHz}}{40}=400$ kHz.
* **Geiger gate (Timer1, CTC):** $f=\dfrac{f_{CPU}}{N\,(1+\mathrm{OCR1A})}=\dfrac{16\cdot10^6}{256\cdot62\,500}=1.000$ Hz.
  **⚠ correction:** the 2022 overflow pre-load of 31 250 gives $(65\,536-31\,250)\cdot16\ \mu\text{s}=0.549$ s, not 1 s.
* **RPi pulse mapping:** stick $v\in[0,100]$ → pulse $=1000+10v$ µs. At 50 Hz the duty cycle is
  $D=\text{pulse}/20\ \text{ms}=(5v+500)/100\ \%$, which is exactly the `ChangeDutyCycle` formula of the thesis.
* **ESC refresh:** 250 Hz, pulse 1100–2000 µs when armed, 1000 µs when disarmed.
* **Battery compensation:** $u' = u\left(1+\dfrac{12.40\ \text{V}-V_{bat}}{35\ \text{V}}\right)$.

## 10. Sensor models

### 10.1 MQ-2 (SnO₂ chemiresistor)

$$
V_{out}=V_C\frac{R_L}{R_S+R_L}\ \Rightarrow\ R_S=R_L\frac{V_C-V_{out}}{V_{out}},\qquad
R_0=\frac{R_S^{\,\text{clean air}}}{9.83},\qquad
c=10^{\frac{\log_{10}(R_S/R_0)-y_0}{m}+x_0}\ \text{ppm}
$$

| gas | $x_0$ | $y_0$ | $m$ |
|---|---|---|---|
| LPG | 2.30 | 0.21 | −0.47 |
| CO | 2.30 | 0.72 | −0.34 |
| smoke | 2.30 | 0.53 | −0.44 |

These fits come from the datasheet's log-log curves and are only approximate (±30 %).
The sensor also responds to humidity and temperature. It indicates trends and is **not** a calibrated gas analyser.

### 10.2 Geiger–Müller counter

Counts in a window $t$ are Poisson distributed: $P(N)=\dfrac{\lambda^N e^{-\lambda}}{N!}$, with
$\sigma_N=\sqrt N$ and relative uncertainty $1/\sqrt N$.

* dead-time (non-paralysable) correction: $n_{true}=\dfrac{n}{1-n\tau_d}$, with $\tau_d \approx 190\ \mu$s for the SBM-20;
* dose-rate estimate: $\dot H\ [\mu\text{Sv/h}] = \text{CPM}\cdot CF$, with $CF\approx0.0057$ for the SBM-20 (Cs-137) and $\approx0.0081$ for the J305;
* the firmware sums 60 one-second gates. At a natural background of ~20 CPM that gives
  $\sigma/N = 1/\sqrt{20}=22\ \%$, so a reading of $0.11\pm0.03\ \mu$Sv/h.

## 11. Fail-safe descent

With no altitude measurement, the fail-safe can only ramp the throttle down open-loop.
Simulated from 30 m (linear drag model):

| strategy | time to ground | impact speed |
|---|---|---|
| ramp 250 µs/s (1 µs per loop) | 3.5 s | 19.8 m/s |
| ramp 25 µs/s (new default) | 6.9 s | 11.4 m/s |
| *proposed:* barometer + vertical-speed PI loop, −1 m/s | 30 s | **1.2 m/s** |

Conclusion: a barometric altimeter (e.g. BMP280 on the same I²C bus) is the most valuable
hardware addition for safe operation beyond visual line of sight.

---
*References:* T. Luukkonen, *Modelling and control of quadcopter*, Aalto Univ., 2011 ·
InvenSense MPU-6000/6050 Product Specification rev. 3.4 · Hanwei MQ-2 datasheet ·
Microchip ATmega328P datasheet · R. Beard & T. McLain, *Small Unmanned Aircraft*, Princeton UP, 2012 ·
G. F. Knoll, *Radiation Detection and Measurement*, 4th ed., Wiley, 2010.
