/**
*   Swerve subsystem declarations for FRC #8193's FRC 2026 season chassis.
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

#include <frc2/command/SubsystemBase.h>
#include <functional>
#include <stingers/swerve.hpp>

namespace stingers::swerve {

class SwerveSubsystem : public frc2::SubsystemBase {
public:
  /**
   * Creates a new swerve subsystem using the configuration defined in `cpp/stingers/config.cpp`.
   */
  SwerveSubsystem();

  /**
   * Sets the target speed and angular speed for the swerve drive.
   */
  void drive_framespace(units::velocity::meters_per_second_t x,
                        units::velocity::meters_per_second_t y,
                        units::angular_velocity::radians_per_second_t t);

  /**
   * Creates a command that gets the target speed and angular speed for the  swerve from a lambda
   * and sets the swerve parameters
   */
  frc2::CommandPtr
  drive_command(std::function<units::velocity::meters_per_second_t()> speed_x,
                std::function<units::velocity::meters_per_second_t()> speed_y,
                std::function<units::angular_velocity::radians_per_second_t()> speed_r);

  void InitSendable(wpi::SendableBuilder &builder) override;

private:
  SwerveDrive drive;
};
} // namespace stingers::swerve
