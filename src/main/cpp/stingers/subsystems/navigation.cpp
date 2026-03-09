/**
*   Navigation subsystem definitions for FRC #8193's FRC 2026 season chassis.
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

#include <frc/smartdashboard/SmartDashboard.h>
#include <stingers/subsystems/navigation.hpp>
#include <stingers/util.hpp>

namespace stingers {

NavigationSubsystem::NavigationSubsystem(const swerve::SwerveSubsystem &drive) : drive(drive), filter(make_q_cv(loop_time, (float)robot_linear_accel), {}) {
  frc::SmartDashboard::PutData("Field", &this->field);
}

glm::vec2 NavigationSubsystem::get_frame_position() const {
  return glm::vec2(this->filter.state.x, this->filter.state.y);
}

glm::vec2 NavigationSubsystem::get_frame_velocity_fieldspace() const {
  return glm::vec2(this->filter.state.z, this->filter.state.w);
}

void NavigationSubsystem::Periodic() {
  std::vector<std::reference_wrapper<const KalmanSensor>> sensors = {};

  sensors.push_back(this->drive.get_velocity_sensor());

  this->filter.update(sensors, loop_time);

  this->field.SetRobotPose(units::meter_t(this->filter.state.x), units::meter_t(this->filter.state.y), frc::Rotation2d());
  frc::SmartDashboard::PutNumber("robot vx", this->drive.get_velocity_sensor().z().x);
  frc::SmartDashboard::PutNumber("robot x", this->filter.state.x);
}
}
