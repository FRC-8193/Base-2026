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

#include <cmath>
#include <iostream>
#include <stingers/swerve/motors.hpp>
#include <frc/smartdashboard/SmartDashboard.h>

namespace stingers::swerve::motors {

void TalonFxDriveMotor::set_ground_speed_setpoint(units::velocity::meters_per_second_t speed) {
  units::meter_t wheel_circ = this->diameter * M_PI;
  float wheel_rps = (float)speed / (float)wheel_circ;
  float motor_rps = wheel_rps / this->ratio;
  auto ctr = ctre::phoenix6::controls::VelocityVoltage(
      units::angular_velocity::turns_per_second_t(motor_rps));
  auto stc = this->motor.SetControl(ctr);
  if (!stc.IsOK()) {
    std::cerr << "Error/Warning setting drive motor control" << std::endl;
  }
}

units::velocity::meters_per_second_t TalonFxDriveMotor::get_ground_speed_real() {
  units::meter_t wheel_circ = this->diameter * M_PI;
  // hehe hopefully no fail
  float motor_rps = this->motor.GetRotorVelocity().GetValue().to<float>();
  float wheel_rps = motor_rps * this->ratio;
  return units::velocity::meters_per_second_t(wheel_rps * (float)wheel_circ);
}

void TalonFxDriveMotor::update_sim(double dt) {
  double motor_voltage = this->motor.GetMotorVoltage().GetValue().to<double>();
  // todo
}

void TalonFxTurnMotor::optimize_angle(units::angle::radian_t &new_angle, units::velocity::meters_per_second_t &new_speed) {
  auto sts = this->motor.GetPosition();
  if (!sts.GetStatus().IsOK()) {
    std::cerr << "Error/Warning retrieving turn motor position" << std::endl;
    return;
  }
  auto cur_angle = units::angle::radian_t(sts.GetValue());
  auto half_turn = 3.1415926535_rad;

  double dot = cos((double)(new_angle-cur_angle));

  new_speed *= dot;

  if (dot < 0.0) {
    new_angle += half_turn;
  }

  while (new_angle - cur_angle > half_turn) { new_angle -= 2.0*half_turn; }
  while (new_angle - cur_angle < -half_turn) { new_angle += 2.0*half_turn; }
}

void TalonFxTurnMotor::set_angle_setpoint_modspace(units::angle::radian_t angle) {
  auto ctr = ctre::phoenix6::controls::PositionVoltage(angle);
  auto stc = this->motor.SetControl(ctr);
  if (!stc.IsOK()) {
    std::cerr << "Error/Warning setting turn motor control" << std::endl;
  }
}

units::angle::radian_t TalonFxTurnMotor::get_angle_real() {
  return this->motor.GetPosition().GetValue();
}

void TalonFxTurnMotor::update_sim(double dt) {
  // todo
}
} // namespace stingers::swerve::motors
