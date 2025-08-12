/**
*   Swerve drive motor declarations for FRC #8193's FRC 2026 season chassis.
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

#include "units/angle.h"
#include "units/velocity.h"
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <stingers/swerve.hpp>

namespace stingers::swerve::motors {

class TalonFxDriveMotor : public DriveMotor {
public:
  TalonFxDriveMotor(int id, float ratio, units::meter_t diameter, float kp, float ki, float kd)
      : motor(id), ratio(ratio), diameter(diameter) {
    ctre::phoenix6::configs::Slot0Configs pid_cfg;
    pid_cfg.kP = kp;
    pid_cfg.kI = ki;
    pid_cfg.kD = kd;

    ctre::phoenix6::configs::FeedbackConfigs fb_cfg;
    fb_cfg.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RotorSensor;

    this->motor.GetConfigurator().Apply(fb_cfg);
    this->motor.GetConfigurator().Apply(pid_cfg);
  }

  void set_ground_speed_setpoint(units::velocity::meters_per_second_t speed) override;

  ~TalonFxDriveMotor() override = default;

private:
  ctre::phoenix6::hardware::TalonFX motor;
  float ratio;
  units::meter_t diameter;
};

class TalonFxTurnMotor : public TurnMotor {
public:
  TalonFxTurnMotor(int id, float ratio, float kp, float ki, float kd)
      : motor(id), encoder(id + 1), ratio(ratio) {
    ctre::phoenix6::configs::Slot0Configs pid_cfg;
    pid_cfg.kP = kp;
    pid_cfg.kI = ki;
    pid_cfg.kD = kd;

    ctre::phoenix6::configs::FeedbackConfigs fb_cfg;
    fb_cfg.FeedbackRemoteSensorID = id + 1;
    fb_cfg.FeedbackSensorSource =
        ctre::phoenix6::signals::FeedbackSensorSourceValue::RemoteCANcoder;
    fb_cfg.RotorToSensorRatio = ratio;

    this->motor.GetConfigurator().Apply(fb_cfg);
    this->motor.GetConfigurator().Apply(pid_cfg);
  }

  void optimize_angle(units::angle::radian_t& new_angle, units::velocity::meters_per_second_t& new_speed) override;

  void set_angle_setpoint_modspace(units::angle::radian_t angle) override;

  ~TalonFxTurnMotor() override = default;

private:
  ctre::phoenix6::hardware::TalonFX motor;
  ctre::phoenix6::hardware::CANcoder encoder;
  float ratio;
};

} // namespace stingers::swerve::motors
