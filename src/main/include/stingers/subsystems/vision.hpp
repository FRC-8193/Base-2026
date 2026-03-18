/**
*   Vision subsystem declarations for FRC #8193's FRC 2026 season chassis.
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

#include <photon/PhotonPoseEstimator.h>
#include <photon/PhotonUtils.h>
#include <stingers/math/kalman.hpp>
#include <units/time.h>
#include <vector>
#include <string>

namespace stingers {

class VisionPositionSensor : public KalmanSensor {
public:
  VisionPositionSensor(std::string camera_name, frc::AprilTagFieldLayout layout, frc::Transform3d robot_to_cam) : camera(camera_name), pose_estimator(layout, robot_to_cam) {}

  virtual glm::vec2 z() const override {
    return this->last_z;
  }

  float yaw() const {
    return this->last_yaw;
  }

  units::second_t timestamp() const {
    return this->last_timestamp;
  }

  virtual glm::mat4x2 H() const override {
    return {
      {1,0},
      {0,1},
      {0,0},
      {0,0}
    };
  }

  virtual glm::mat2x2 R() const override {
    return this->last_R;
  }

  // returns true if update was successful and a valid measurement is ready
  bool update();

private:
  photon::PhotonCamera camera;
  photon::PhotonPoseEstimator pose_estimator;

  glm::vec2 last_z = glm::vec2(0,0);
  float last_yaw = 0.0f;
  units::second_t last_timestamp;
  glm::mat4x2 last_R = glm::mat2x2(std::numeric_limits<float>::max());
};

struct VisionConfigs {
  std::vector<std::string> camera_names;
  std::vector<frc::Transform3d> robot_to_camera_transforms;
  frc::AprilTagFieldLayout field_layout;
};

const extern VisionConfigs vision_configs;

class VisionSubsystem {
public:
  VisionSubsystem();

  // updates and returns all sensors with valid measurements
  std::vector<std::reference_wrapper<const VisionPositionSensor>> get_sensors();
private:
  std::vector<VisionPositionSensor> sensors;
};
}
