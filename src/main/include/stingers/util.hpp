/**
*   Utility functions and constants for FRC #8193's FRC 2026 season chassis.
*
*   Copyright (C) 2025 Frederick Ziola, et al. (New Lothrop Robotics)
*   This program is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.

*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.

*   You should have received a copy of the GNU General Public License
*   along with this program.  If not, see <https://www.gnu.org/licenses/>.
*
*   Contact us: robotics@newlothrop.k12.mi.us
*/

#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <glm/glm.hpp>

namespace stingers {

constexpr double loop_time = 0.02;

inline double motor_tau(double Kt, double R, double V, double Ke, double omega) {
  return (Kt / R) * (V - Ke * omega);
}

inline double est_motor_torque(ctre::phoenix6::hardware::TalonFX &motor) {
  // Commanded motor/output voltage
  double motor_voltage =
      motor.GetMotorVoltage().GetValue().to<double>();

  // Use bus voltage to clamp (keeps the motor model honest)
  const double supply_voltage =
      motor.GetSupplyVoltage().GetValue().to<double>();

  motor_voltage = std::clamp(
      motor_voltage,
      -supply_voltage,
       supply_voltage);

  double omega = motor.GetRotorVelocity().GetValueAsDouble() * 2.0 * M_PI;

  // kraken x60 motor constants (TODO: update this to be dynamic when changing to 2026 [maybe])
  static constexpr double R  = 0.033;   // ohms (approx)
  static constexpr double Kt = 0.0191;  // N·m/A
  static constexpr double Ke = 0.0191;  // V/(rad/s) consistent with Kv

  return motor_tau(Kt, R, motor_voltage, Ke, omega);
}

constexpr glm::mat4 make_q_cv(float dt, float sigma_a) {
    const float dt2 = dt * dt;
    const float dt3 = dt2 * dt;
    const float dt4 = dt2 * dt2;

    const float q = sigma_a * sigma_a;

    // GLM is column-major: mat[col][row]
    glm::mat4 Q(0.0f);

    Q[0][0] = dt4 / 4.0f;
    Q[0][2] = dt3 / 2.0f;

    Q[1][1] = dt4 / 4.0f;
    Q[1][3] = dt3 / 2.0f;

    Q[2][0] = dt3 / 2.0f;
    Q[2][2] = dt2;

    Q[3][1] = dt3 / 2.0f;
    Q[3][3] = dt2;

    return q * Q;
}
}
