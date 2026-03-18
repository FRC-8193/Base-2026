/**
*   Path following command declarations for FRC #8193's FRC 2026 season chassis.
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

#include <stingers/math/path.hpp>
#include <stingers/subsystems/swerve.hpp>
#include <stingers/subsystems/navigation.hpp>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <stingers/math/pid.hpp>
#include <memory>

namespace stingers {

struct FollowPathConfig {
  float aggressiveness = 1.0; // % of max speed to use
  bool stop_at_end = false;
};

class FollowPath : public frc2::CommandHelper<frc2::Command, FollowPath> {
public:
  FollowPath(swerve::SwerveSubsystem &swerve, NavigationSubsystem &navi, std::unique_ptr<math::Path> path, const FollowPathConfig &config) : swerve(swerve), navi(navi), path(std::move(path)), position_pid(10.0, 0.0, 0.0) {
    this->AddRequirements(&this->swerve);
    this->aggressiveness = config.aggressiveness;
    this->stop_at_end = config.stop_at_end;
  }

  virtual void Execute() override;
  virtual inline bool IsFinished() override { return this->t > 0.99f; }

private:
  float calc_vmax(float t);
  float find_nearest_t();

  swerve::SwerveSubsystem &swerve;
  NavigationSubsystem &navi;
  std::unique_ptr<math::Path> path;

  PIDController<glm::vec2, float> position_pid;

  float aggressiveness;
  bool stop_at_end;

  float t = 0.0f;
};

}
