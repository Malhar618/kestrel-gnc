#pragma once

namespace gnc {

/// One classical 4th-order Runge-Kutta step of x' = f(t, x).
/// State only needs vector-space operations: State + State and T * State.
/// Local error is O(dt^5) and global error O(dt^4), so halving dt cuts the
/// error about 16x (tests/unit/test_rk4.cpp checks exactly that).
template <typename T, typename State, typename Deriv>
constexpr State rk4_step(Deriv&& f, T t, const State& x, T dt) {
  const T half = dt / T(2);
  const State k1 = f(t, x);
  const State k2 = f(t + half, x + half * k1);
  const State k3 = f(t + half, x + half * k2);
  const State k4 = f(t + dt, x + dt * k3);
  return x + (dt / T(6)) * (k1 + T(2) * k2 + T(2) * k3 + k4);
}

}  // namespace gnc
