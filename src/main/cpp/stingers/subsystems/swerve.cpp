/**
*   Swerve subsystem implementations for FRC #8193's FRC 2026 season chassis.
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

#include <frc2/command/Commands.h>
#include <stingers/subsystems/swerve.hpp>
#include <iostream>

namespace stingers::swerve {

SwerveSubsystem::SwerveSubsystem() : drive(swerve_config), velocity_sensor(drive) {
  this->SetDefaultCommand(
      this->Run([this]() { this->drive_framespace(0_mps, 0_mps, 0_rad_per_s); }));
}

void SwerveSubsystem::drive_framespace(units::velocity::meters_per_second_t x,
                                       units::velocity::meters_per_second_t y,
                                       units::angular_velocity::radians_per_second_t t) {
  this->drive.set_velocity_setpoint_framespace(x, y, t);
}

frc2::CommandPtr SwerveSubsystem::drive_command(
    std::function<units::velocity::meters_per_second_t()> speed_x,
    std::function<units::velocity::meters_per_second_t()> speed_y,
    std::function<units::angular_velocity::radians_per_second_t()> speed_r) {
  return this->Run([speed_x, speed_y, speed_r, this] {
    this->drive.set_velocity_setpoint_framespace(speed_x(), speed_y(), speed_r());
  });
}

void SwerveSubsystem::InitSendable(wpi::SendableBuilder &builder) {
  builder.SetSmartDashboardType("SwerveDrive");

  static const char *names[] = {
      "Front Left Angle",  "Front Left Velocity",

      "Front Right Angle", "Front Right Velocity",

      "Back Left Angle",   "Back Left Velocity",

      "Back Right Angle",  "Back Right Velocity",

  };

  // not really sure how to get the data i need here
}
} // namespace stingers::swerve
