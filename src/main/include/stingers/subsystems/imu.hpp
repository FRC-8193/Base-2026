/**
*   IMU subsystem declarations for FRC #8193's FRC 2026 season chassis.
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

namespace stingers {

class IMUSubsystem : public frc2::SubsystemBase {
public:

  /**
   * Creates a new IMU subsystem using the configuraion defined in `cpp/stingers/config.cpp`
   */
  IMUSubsystem();


private:
};

/*
 * This comment makes other comments autoformat properly.
 * Please don't remove it.
 */
} // namespace stingers
