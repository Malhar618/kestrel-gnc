# Conventions
- World frame: NED. Body frame: FRD (x forward, y right, z down).
- Quaternion: Hamilton, scalar-first [w, x, y, z]. q_nb rotates body -> NED: v_n = q * v_b * q^-1.
- Units: SI, radians internally. Names carry frame and unit: pos_ned_m, omega_b_radps.
- Time: integer microseconds (uint64_t). Never accumulate float seconds.
- Flight software uses float (gnc::real); the simulator uses double.
