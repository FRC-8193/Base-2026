// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/smartdashboard/SmartDashboard.h>
#include <RobotContainer.h>
#include <frc2/command/Commands.h>
#include <iostream>
#include <stingers/commands/follow_path.hpp>
#include <stingers/math/linear_path.hpp>
#include <stingers/commands/deploy_intake.hpp>
#include <stingers/commands/toggle_roller.hpp>
#include <frc/DriverStation.h>

RobotContainer::RobotContainer() : swerve(navigation), navigation(swerve, imu), driver(0) {
  ConfigureBindings();

  frc::SmartDashboard::PutData("swerve", &this->swerve);
}

void RobotContainer::ConfigureBindings() {
  this->swerve.SetDefaultCommand(this->swerve.drive_command(
      [this] { return units::meters_per_second_t(-this->driver.GetHID().GetRawAxis(0))*2.0; },
      [this] { return units::meters_per_second_t(-this->driver.GetHID().GetRawAxis(1))*2.0; },
      [this] { return units::radians_per_second_t(this->driver.GetHID().GetRawAxis(2)); }));

  stingers::DeployIntakeCommand(&this->intake, false).Schedule();

  this->driver.Button(7)
    .OnTrue(frc2::CommandPtr(stingers::DeployIntakeCommand(&this->intake, true)));
  this->driver.Button(8)
    .OnFalse(frc2::CommandPtr(stingers::DeployIntakeCommand(&this->intake, false)));

  this->driver.Button(2)
    .OnTrue(frc2::CommandPtr(stingers::ToggleRollerCommand(&this->intake, true)))
    .OnFalse(frc2::CommandPtr(stingers::ToggleRollerCommand(&this->intake, false)));

  auto alliance = frc::DriverStation::GetAlliance();
  bool is_blue = alliance == frc::DriverStation::Alliance::kBlue;
  float driver_forward = is_blue ? 0.0f : M_PI;
  glm::vec2 hub_position = glm::vec2(is_blue ? 4.62 : 11.91, 4.0f);
  
  this->driver.Button(9)
    .OnTrue(frc2::CommandPtr(this->turret.aim_at_command(this->navigation, [hub_position] { return hub_position; })));
  this->driver.Button(10)
    .OnTrue(frc2::CommandPtr(this->turret.aim_command(this->navigation, [driver_forward] { return units::radian_t(M_PI + driver_forward); })));
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
