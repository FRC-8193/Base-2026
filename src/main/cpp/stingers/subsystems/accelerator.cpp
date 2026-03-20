/**
*   Accelerator subsystem definitions for FRC #8193's FRC 2026 season chassis.
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

#include <stingers/subsystems/accelerator.hpp>

namespace stingers {

AcceleratorSubsystem::AcceleratorSubsystem() : pre_accelerator(pre_accelerator_canid, rev::spark::SparkLowLevel::MotorType::kBrushless), accelerator(accelerator_canid, rev::spark::SparkLowLevel::MotorType::kBrushless) {
}

frc2::CommandPtr AcceleratorSubsystem::run_command(float speed) {
  return this->run_command([speed] { return speed; });
}

frc2::CommandPtr AcceleratorSubsystem::run_command(std::function<float()> speed) {
  return this->Run([this, speed] {
    this->set_speed(speed());
  });
}

void AcceleratorSubsystem::set_speed(float speed) {
  this->pre_accelerator.Set(-speed);
  this->accelerator.Set(speed);
}

}
