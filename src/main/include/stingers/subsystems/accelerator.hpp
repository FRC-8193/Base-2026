/**
*   Accelerator subsystem declarations for FRC #8193's FRC 2026 season chassis.
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

#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <functional>
#include <rev/SparkFlex.h>

namespace stingers {

const extern int pre_accelerator_canid;
const extern int accelerator_canid;

class AcceleratorSubsystem : public frc2::Subsystem {
public:
  AcceleratorSubsystem();

  frc2::CommandPtr run_command(float speed);
  frc2::CommandPtr run_command(std::function<float()> speed);
  void set_speed(float speed);

private:
  rev::spark::SparkFlex pre_accelerator, accelerator;
};

}
