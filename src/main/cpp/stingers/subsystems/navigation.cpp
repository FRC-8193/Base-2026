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

NavigationSubsystem::NavigationSubsystem(swerve::SwerveSubsystem &drive, IMUSubsystem &imu) : drive(drive), imu(imu), filter(make_q_cv(loop_time, (float)robot_linear_accel), {}) {
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

  this->drive.get_velocity_sensor().set_yaw(this->yaw);
  sensors.push_back(this->drive.get_velocity_sensor());

  this->yaw -= loop_time * (float)this->imu.get_yaw_rate();

  for (const auto &sensor : this->vision.get_sensors()) {
    // use vision sensors for yaw update
    float sensor_yaw = sensor.get().yaw();
    units::second_t age = frc::Timer::GetFPGATimestamp() - sensor.get().timestamp();

    float alpha = std::min(0.5f, 0.002f / (float)age);
    float error = angle_diff(sensor_yaw, this->yaw);

    if (!this->has_yaw) {
      alpha = 1.0;
      this->has_yaw = true;
    }

    this->yaw += error * alpha * loop_time;

    sensors.push_back(sensor);
  }

  this->filter.update(sensors, loop_time);

  this->field.SetRobotPose(units::meter_t(this->filter.state.x), units::meter_t(this->filter.state.y), frc::Rotation2d(units::radian_t(this->yaw)));
  frc::SmartDashboard::PutNumber("robot vx", this->drive.get_velocity_sensor().z().x);
  frc::SmartDashboard::PutNumber("robot x", this->filter.state.x);
}
}
