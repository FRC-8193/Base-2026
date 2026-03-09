/**
*   Drive subsystem declarations for FRC #8193's FRC 2026 season chassis.
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
#include <stingers/subsystems/swerve.hpp>
#include <stingers/math/path.hpp>

namespace stingers {

class DriveSubsystem : public frc2::SubsystemBase {
public:
  DriveSubsystem();

  frc2::CommandPtr follow_path(const math::Path &path);

  swerve::SwerveSubsystem &get_swerve_subsystem() { return this->swerve_subsystem; }
private:
  swerve::SwerveSubsystem swerve_subsystem = {};
};
}
