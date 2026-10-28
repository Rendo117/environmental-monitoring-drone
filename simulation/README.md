# Simulation

`quad_sim.py` integrates the nonlinear Newton–Euler model of the quadcopter at 2 kHz. A Python replica
of the firmware runs at 250 Hz alongside it: integer IMU readings, complementary filter,
low-pass filters, cascaded angle-P/rate-PID, mixer, ESC limits and fail-safe ramp.

```bash
pip install numpy matplotlib
python3 quad_sim.py
```

Outputs (also copied to `docs/assets/img/` for the website):

| file | experiment |
|---|---|
| `results/sim-roll-step.png` | roll-stick step 1500→1600 µs |
| `results/sim-disturbance.png` | 0.4 N·m / 50 ms gust while hovering |
| `results/sim-bode.png` | open-loop Bode plot of the roll-rate loop, stability margins |
| `results/sim-pitch-sign-bug.png` | 2022 pitch sign error vs corrected firmware |
| `results/sim-complementary.png` | attitude estimator vs gyro-only and accelerometer-only |
| `results/sim-failsafe.png` | fail-safe descent strategies from 30 m |
| `results/sim-geiger-uncertainty.png` | Poisson counting uncertainty |
| `results/sim-mq2-curves.png` | MQ-2 datasheet curves |
| `results/summary.json` | all numeric results |

The airframe constants in `Params` are estimates for an F450-class frame. Replace them with measured
values (mass, thrust curve, bifilar-pendulum inertia) for quantitative conclusions.
