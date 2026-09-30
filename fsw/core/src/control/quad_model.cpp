#include "gnc/control/quad_model.hpp"

#include <cmath>

namespace gnc {

QuadModel default_quad_model() {
  QuadModel m;
  m.mass_kg = 1.5f;
  m.inertia_b_kgm2 = diag(Vec3f{0.020f, 0.020f, 0.035f});
  const real a = 0.225f / std::sqrt(2.0f);  // arm length 0.225 m at 45 deg
  m.rotor_pos_b_m = {{{a, a, 0}, {-a, -a, 0}, {a, -a, 0}, {-a, a, 0}}};
  m.rotor_spin = {1, 1, -1, -1};
  m.thrust_coeff = 1.0e-5f;
  m.torque_coeff = 1.6e-7f;
  m.rotor_speed_max_radps = 1100.0f;
  return m;
}

}  // namespace gnc
