/**
*   Turret subsystem definitions for FRC #8193's FRC 2026 season chassis.
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

#include <stingers/subsystems/turret.hpp>
#include <ctre/phoenix6/controls/MotionMagicVoltage.hpp>

namespace stingers {

TurretSubsystem::TurretSubsystem() : aim_motor(turret_config.aim_id), hood_motor(turret_config.hood_id, rev::spark::SparkLowLevel::MotorType::kBrushless) {
}


frc2::CommandPtr TurretSubsystem::aim_at_command(NavigationSubsystem &navigation, std::function<glm::vec2()> aim_position) {
  return this->aim_command(navigation, [this, &navigation, aim_position] {
    glm::vec2 pos = navigation.get_frame_position();
    glm::vec2 targ = aim_position();

    glm::vec2 delta = targ - pos;

    navigation.get_field().GetObject("aim_location")->SetPose(frc::Pose2d{units::meter_t{targ.x}, units::meter_t{targ.y}, frc::Rotation2d{units::radian_t(glm::atan(delta.y, delta.x))}});
    return units::radian_t(glm::atan(delta.y, delta.x));
  });
}

frc2::CommandPtr TurretSubsystem::aim_command(NavigationSubsystem &navigation, std::function<units::angle::radian_t()> angle) {
  return this->Run([this, &navigation, angle] {
    this->set_aim_angle(units::angle::radian_t((float)angle() - navigation.get_yaw()));
  });
}
void TurretSubsystem::set_aim_angle(units::radian_t angle) {
  auto ctr = ctre::phoenix6::controls::MotionMagicVoltage(units::radian_t(fmodf((float)angle, 2.0 * M_PI))).WithSlot(0);
  this->aim_motor.SetControl(ctr);
}
}
