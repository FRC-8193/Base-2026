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
#include <ctre/phoenix6/sim/TalonFXSimState.hpp>
#include <frc/smartdashboard/SmartDashboard.h>

namespace stingers::swerve::motors {

static double motor_tau(double Kt, double R, double V, double Ke, double omega) {
  return (Kt / R) * (V - Ke * omega);
}

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
  // TODO: set simstate motor type when switching to 2026
  
  // Commanded motor/output voltage
  double motor_voltage =
      this->motor.GetMotorVoltage().GetValue().to<double>();

  // Use bus voltage to clamp (keeps the motor model honest)
  const double supply_voltage =
      this->motor.GetSupplyVoltage().GetValue().to<double>();

  motor_voltage = std::clamp(
      motor_voltage,
      -supply_voltage,
       supply_voltage);

  double omega = this->motor.GetRotorVelocity().GetValueAsDouble() * 2.0 * M_PI;

  if (omega != omega) omega = 0.0;

  // kraken x60 motor constants
  static constexpr double R  = 0.033;   // ohms (approx)
  static constexpr double Kt = 0.0191;  // N·m/A
  static constexpr double Ke = 0.0191;  // V/(rad/s) consistent with Kv

  static constexpr double robot_mass = 50; // kg
  static constexpr double wheel_mass = robot_mass / 4.0; // just assume each wheel moves 1/4 of the robot

  double rotor_torque = motor_tau(Kt, R, motor_voltage, Ke, omega);
  double wheel_torque = rotor_torque / this->ratio;

  double ground_force = wheel_torque / (0.5 * (double)this->diameter);

  double ground_accel = ground_force / wheel_mass;

  double wheel_accel = ground_accel / (0.5 * (double)this->diameter);
  double rotor_accel = wheel_accel / this->ratio;

  std::cout << "rotor_tau=" << rotor_torque
            << " wheel_tau=" << wheel_torque
            << " ground_F=" << ground_force
            << " ground_a=" << ground_accel
            << " wheel_alpha=" << wheel_accel
            << " rotor_alpha=" << rotor_accel
            << std::endl;

  this->motor.GetSimState().SetRotorAcceleration(units::radians_per_second_squared_t(rotor_accel));
  this->motor.GetSimState().SetRotorVelocity(units::radians_per_second_t(omega + rotor_accel * dt));
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
