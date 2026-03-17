/**
*   Vision subsystem definitions for FRC #8193's FRC 2026 season chassis.
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

#include <stingers/subsystems/vision.hpp>
#include <cassert>

namespace stingers {

bool VisionPositionSensor::update() {
  auto results = this->camera.GetAllUnreadResults();
  if (results.empty()) return false;

  auto &newest = results.back();

  bool is_multi_tag = true;

  auto vision_est = this->pose_estimator.EstimateCoprocMultiTagPose(newest);

  if (!vision_est) {
    is_multi_tag = false;
    vision_est = this->pose_estimator.EstimateLowestAmbiguityPose(newest);
  }

  if (!vision_est) return false;

  frc::Pose2d pose = vision_est->estimatedPose.ToPose2d();

  this->last_z = { pose.X().value(), pose.Y().value() };

  double distance = newest.GetBestTarget().GetBestCameraToTarget().Translation().Norm().value();

  float base_std_dev = is_multi_tag ? 0.05f : 0.25f;

  // Error typically scales with the square of distance
  float std_dev = base_std_dev * static_cast<float>(distance * distance);

  float variance = std_dev * std_dev;
  this->last_R = glm::mat2(
      variance, 0.0f,
      0.0f,     variance
  );
  return true;
}

VisionSubsystem::VisionSubsystem() {
  assert(vision_configs.camera_names.size() == vision_configs.robot_to_camera_transforms.size());

  for (unsigned int i = 0; i < vision_configs.camera_names.size(); ++i) {
    this->sensors.push_back(VisionPositionSensor(
      vision_configs.camera_names[i],
      vision_configs.field_layout,
      vision_configs.robot_to_camera_transforms[i]
    ));
  }
}

std::vector<std::reference_wrapper<const VisionPositionSensor>> VisionSubsystem::get_sensors() {
  std::vector<std::reference_wrapper<const VisionPositionSensor>> out_sensors;
  for (auto &sensor : this->sensors) {
    if (sensor.update()) {
      out_sensors.push_back(sensor);
    }
  }
  return out_sensors;
}

}
