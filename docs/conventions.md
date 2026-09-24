# Conventions

- World frame: NED. Body frame: FRD (x forward, y right, z down).
- Quaternion: Hamilton, scalar-first [w, x, y, z]. q_nb rotates body -> NED: v_n = q * v_b * q^-1.
- Units: SI, radians internally. Names carry frame and unit: pos_ned_m, omega_b_radps.
- Time: integer microseconds (uint64_t). Never accumulate float seconds.
- Flight software uses float (gnc::real); the simulator uses double.

## Rotations in code (`fsw/core/include/gnc/math/`)

| Quantity | Formula | Function |
|---|---|---|
| Hamilton product | ij = k, jk = i, ki = j, i² = j² = k² = ijk = −1 | `operator*(Quat, Quat)` |
| Composition | q_nb = q_na ⊗ q_ab (right factor applies first) | `a * b` |
| Rotate a vector | v_n = q ⊗ [0, v_b] ⊗ q* | `rotate(q, v)` |
| DCM | C_nb with v_n = C_nb v_b, and C(q₁ ⊗ q₂) = C(q₁) C(q₂) | `to_dcm`, `from_dcm` |
| Exponential map | exp(θ) = [cos(‖θ‖/2), sin(‖θ‖/2) θ/‖θ‖] | `from_rotation_vector` |
| Logarithm map | inverse of exp, angle in [0, π] | `to_rotation_vector` |
| Kinematics | q̇ = ½ q ⊗ [0, ω_b], ω_b = body rate in body axes (gyro) | `derivative` |
| Discrete propagation | q_{k+1} = q_k ⊗ exp(ω_b Δt), exact for constant ω_b | `integrate` |
| Euler 3-2-1 | q_nb = q_z(yaw) ⊗ q_y(pitch) ⊗ q_x(roll) | `from_euler321`, `to_euler321` |

Physical checks pinned by tests (`tests/unit/test_quat.cpp`):

- +90° yaw: nose north → east, right wing east → south.
- +90° pitch: nose → up, which is −z in NED.
- +90° roll: right wing → down (+z).
- Body rates compose on the right: after a 90° yaw, a body-x rate rolls about east, not north.

Euler angles are for logs, plots and human input only. At pitch = ±90° (gimbal lock)
only yaw − roll (pitch +90°) or yaw + roll (pitch −90°) is observable; `to_euler321`
then reports roll = 0.

## Quadrotor plant (`sim/`)

Motor numbering follows PX4's Quad X, so logs and parameters map directly onto a PX4 vehicle:

| Motor | Position | Spin (seen from above) |
|---|---|---|
| 1 | front-right | CCW |
| 2 | rear-left | CCW |
| 3 | front-left | CW |
| 4 | rear-right | CW |

- Thrust T = k_T ω² acts along −z body (up). Reaction torque Q = k_Q ω²: a CCW rotor yaws the airframe nose-right (+z body).
- Motor commands are normalized, u ∈ [0, 1], and set a rotor-speed target u·ω_max reached through a first-order lag.
- Drag is linear in airspeed (velocity minus wind), per body axis, acting at the CG.
- Physics steps at 1 kHz with RK4; time is integer microseconds.

Sign checks pinned by `tests/sim/test_quadrotor.cpp`: front rotors faster → nose up;
right rotors faster → roll left; CCW rotors faster → nose right.
