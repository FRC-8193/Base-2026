// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/smartdashboard/SmartDashboard.h>
#include <RobotContainer.h>
#include <frc2/command/Commands.h>
#include <stingers/commands/follow_path.hpp>
#include <stingers/math/linear_path.hpp>

RobotContainer::RobotContainer() : swerve(imu), navigation(swerve, imu), driver(0) {
  ConfigureBindings();

  frc::SmartDashboard::PutData("swerve", &this->swerve);
}

void RobotContainer::ConfigureBindings() {
  this->swerve.SetDefaultCommand(this->swerve.drive_command(
      [this] { return units::meters_per_second_t(-this->driver.GetRawAxis(0))*2.0; },
      [this] { return units::meters_per_second_t(-this->driver.GetRawAxis(1))*2.0; },
      [this] { return units::radians_per_second_t(this->driver.GetRawAxis(2)); }));

  this->turret.SetDefaultCommand(this->turret.aim_command(this->navigation));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  stingers::FollowPathConfig config = {
    .aggressiveness = 0.25,
    .stop_at_end = true
  };
  return frc2::cmd::RepeatingSequence(
    stingers::FollowPath(this->swerve, this->navigation, std::make_unique<stingers::math::LinearPath>(glm::vec2(0.0, 0.0), glm::vec2(4.0, 0.0)), config).ToPtr(),
    stingers::FollowPath(this->swerve, this->navigation, std::make_unique<stingers::math::LinearPath>(glm::vec2(4.0, 0.0), glm::vec2(4.0, 4.0)), config).ToPtr(),
    stingers::FollowPath(this->swerve, this->navigation, std::make_unique<stingers::math::LinearPath>(glm::vec2(4.0, 4.0), glm::vec2(0.0, 4.0)), config).ToPtr(),
    stingers::FollowPath(this->swerve, this->navigation, std::make_unique<stingers::math::LinearPath>(glm::vec2(0.0, 4.0), glm::vec2(0.0, 0.0)), config).ToPtr()
  );
}
