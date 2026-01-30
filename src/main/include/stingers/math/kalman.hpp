/**
*   Kalman filter declarations for FRC #8193's FRC 2026 season chassis.
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

#include <glm/glm.hpp>
#include <optional>
#include <vector>
#include <functional>

namespace stingers {

class KalmanSensor {
public:
  virtual glm::vec2 z() const = 0;

  virtual glm::mat4x2 H() const = 0; // observation matrix
  virtual glm::mat2x2 R() const = 0; // sensor covariance
};

class KalmanFilter {
public:
  KalmanFilter(glm::mat4x4 Q, std::optional<std::reference_wrapper<const KalmanSensor>> init_sensor);

  // should probably un-inline this later
  void update(std::vector<std::reference_wrapper<const KalmanSensor>> sensors, float dt);

  glm::vec4 state;
  glm::mat4x4 covariance = glm::mat4(1000.0);
  glm::mat4x4 Q;
};

} // namespace stingers
