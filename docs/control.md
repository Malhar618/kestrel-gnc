# Control (`fsw/core/include/gnc/control/`)

Flight software, single precision, no heap. `QuadController::step()` runs the whole
cascade once per call (250 Hz in the simulator).

```mermaid
flowchart LR
  P["Position error"] -->|"P, speed-limited"| V["Velocity setpoint"]
  V -->|"PI (anti-windup)"| A["Acceleration setpoint"]
  A -->|"thrust_to_attitude<br/>tilt ≤ 35°"| Q["Attitude setpoint + thrust"]
  Q -->|"P on log(q⁻¹ ⊗ q_sp)"| W["Body-rate setpoint"]
  W -->|"PID, D on measurement"| AL["Angular acceleration"]
  AL -->|"J α + ω × Jω"| T["Torque + thrust"]
  T -->|"Mixer"| M["Motor commands"]
```

## Pieces

| Piece | What it does | Detail that matters |
|---|---|---|
| `thrust_to_attitude` | acceleration + heading → attitude + thrust | z_b = −F/‖F‖ with F = m(a − g); heading fixes x_b. Tilt limited, 10 % minimum thrust, thrust = F projected on the *current* thrust axis. |
| `attitude_rate_setpoint` | attitude error → body rates | Error = conj(q) ⊗ q_sp, which is in current body axes; the log map takes the short way. Per-axis rate limits. |
| Rate loop (`Pid3`) | rate error → angular acceleration | Derivative on the measurement (no kick on setpoint steps), 30 Hz filter. Torque adds ω × Jω to cancel gyroscopic coupling. |
| `Mixer` | thrust + torque → four motor commands | Inverts the allocation T = Σf, τx = Σ−y·f, τy = Σx·f, τz = Σ spin·(k_Q/k_T)·f, then u = √(f/k_T)/ω_max. |
| `QuadModel` | the flight software's vehicle model | A separate copy of the vehicle parameters; tests check it matches the simulator's defaults. |

## When the motors can't deliver

The mixer gives things up in this order:

1. **Yaw** is scaled down first.
2. **Collective thrust** is shifted as little as possible to keep every rotor in range.
3. **Roll and pitch** are scaled last, keeping their direction.

While the mixer reports saturation, the velocity and rate integrators hold their values;
the velocity integrator also holds while the tilt limit clips the horizontal request
(conditional-integration anti-windup).

## Default gains

Chosen inside-out, each loop a few times slower than the one it drives, then checked in
closed loop, including against a plant with +30 % inertia, 50 % slower motors and −10 %
thrust coefficient:

| Loop | Gains | Why |
|---|---|---|
| Rate (roll, pitch) | kp 22, ki 20, kd 0.2 | Crossover below the 33 rad/s motor pole. kd buys speed on the nominal plant: a 10° roll step settles in 0.40 s instead of 0.51 s. It adds no robustness (worst-case overshoot 8% with it, 7% without). |
| Attitude | kp 6 (roll, pitch), 4 (yaw) | ~3.7× slower than the rate loop. That ratio sets the damping, ζ ≈ ½√(k_rate / k_att); at 2.25 (kp 8 over rate kp 18) the mismatched plant overshot 28%. |
| Velocity | kp 4, ki 1 (horizontal); kp 5, ki 2 (vertical) | |
| Position | kp 1.2 (horizontal), 2 (vertical) | ~3.3× slower than velocity; 5 m/s horizontal and 2 m/s vertical speed limits. |

Closed-loop results on the default plant (`tests/sim/test_closed_loop.cpp`):

| Case | Result |
|---|---|
| 1 m position step | no overshoot, within 2 cm after 2.4 s, peak tilt 22° |
| 10° roll step (attitude mode) | no overshoot, within 0.2° after 0.41 s |
| Recovery from 2.5 m offset and tilted | within 5 cm and 1° after 6 s |
| Steady 3 m/s wind | within 2 cm after 10 s, leaning into the wind by atan(drag/weight) |
| 20 m step | speed and tilt limits engage, no overshoot |
