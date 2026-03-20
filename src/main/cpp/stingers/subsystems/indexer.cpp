/**
*   Indexer subsystem definitions for FRC #8193's FRC 2026 season chassis.
*
*   Copyright (C) 2026 Frederick Ziola, et al. (New Lothrop Robotics)
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

#include <stingers/subsystems/indexer.hpp>

namespace stingers {

IndexerSubsystem::IndexerSubsystem() : motor(indexer_canid) {
}

frc2::CommandPtr IndexerSubsystem::run_command(float speed) {
  return this->run_command([speed] { return speed; });
}

frc2::CommandPtr IndexerSubsystem::run_command(std::function<float()> speed) {
  return this->Run([this, speed] {
    this->set_speed(speed());
  });
}

void IndexerSubsystem::set_speed(float speed) {
  this->motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, speed);
}

}
