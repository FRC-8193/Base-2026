/**
*   Swerve drive motor implementations for FRC #8193's FRC 2026 season chassis.
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

#include "units/angular_velocity.h"
#include <cmath>
#include <iostream>
#include <stingers/swerve_motors.hpp>

namespace stingers::swerve::motors {

void TalonFxDriveMotor::set_ground_speed_setpoint(units::velocity::meters_per_second_t speed) {
  units::meter_t wheel_circ = this->diameter * M_PI;
  float wheel_rps = (float)speed / (float)wheel_circ;
  float motor_rps = wheel_rps / this->ratio;
  auto ctr = ctre::phoenix6::controls::VelocityVoltage(
      units::angular_velocity::radians_per_second_t(motor_rps * (2.0 * M_PI)));
  auto stc = this->motor.SetControl(ctr);
  if (stc.IsError() || stc.IsWarning()) {
    std::cerr << "Error/Warning setting drive motor control" << std::endl;
  }
}

void TalonFxTurnMotor::set_angle_setpoint_modspace(units::angle::radian_t angle) {
  // ah shit i should probably implement this
}
} // namespace stingers::swerve::motors
