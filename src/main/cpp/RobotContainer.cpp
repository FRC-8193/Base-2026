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

RobotContainer::RobotContainer() : swerve(navigation), navigation(swerve, imu), driver(0) {
  ConfigureBindings();

  frc::SmartDashboard::PutData("swerve", &this->swerve);
}

void RobotContainer::ConfigureBindings() {
  this->swerve.SetDefaultCommand(this->swerve.drive_command(
      [this] { return units::meters_per_second_t((this->is_blue() ? 1.0 : -1.0) * this->driver.GetHID().GetRawAxis(0))*5.0; },
      [this] { return units::meters_per_second_t((this->is_blue() ? 1.0 : -1.0) * this->driver.GetHID().GetRawAxis(1))*5.0; },
      [this] { return units::radians_per_second_t(this->driver.GetHID().GetRawAxis(2))*2.5; }));

  stingers::DeployIntakeCommand(&this->intake, false).Schedule();

  //this->driver.Button(7)
  //  .OnTrue(frc2::CommandPtr(stingers::DeployIntakeCommand(&this->intake, true)));
  //this->driver.Button(8)
  //  .OnFalse(frc2::CommandPtr(stingers::DeployIntakeCommand(&this->intake, false)));

  //this->driver.Button(2)
  //  .OnTrue(frc2::CommandPtr(stingers::ToggleRollerCommand(&this->intake, true)))
  //  .OnFalse(frc2::CommandPtr(stingers::ToggleRollerCommand(&this->intake, false)));

  this->driver.Button(9)
    .OnTrue(frc2::CommandPtr(this->turret.aim_at_command(this->navigation, [this] { return this->hub_position(); })));
  this->driver.Button(10)
    .OnTrue(frc2::CommandPtr(this->turret.aim_command(this->navigation, [this] { return units::radian_t(M_PI + this->driver_forward_radians()); })));

  this->driver.Button(1)
    .OnTrue(this->indexer.run_command(-1.0))
    .OnFalse(this->indexer.run_command(0.0));

  this->driver.Button(3)
    .OnTrue(this->accelerator.run_command([this] { return this->driver.GetHID().GetRawAxis(3) * -0.5 + 0.5; }))
    .OnFalse(this->accelerator.run_command(0.0));

  this->driver.Button(4)
    .OnTrue(frc2::cmd::Parallel(this->accelerator.run_command(-1.0), this->indexer.run_command(1.0)))
    .OnFalse(frc2::cmd::Parallel(this->accelerator.run_command(0.0), this->indexer.run_command(0.0)));

  auto turret_func = [this] {
    this->turret.set_hood_angle(0.33 * this->driver.GetHID().GetRawButton(11) + 0.66 * this->driver.GetHID().GetRawButton(12));
   };

  this->driver.Button(11).OnChange(this->turret.Run(turret_func));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  /*stingers::FollowPathConfig config = {
    .aggressiveness = 0.25,
    .stop_at_end = true
  };
  return frc2::cmd::RepeatingSequence(
    stingers::FollowPath(this->swerve, this->navigation, std::make_unique<stingers::math::LinearPath>(glm::vec2(0.0, 0.0), glm::vec2(4.0, 0.0)), config).ToPtr(),
    stingers::FollowPath(this->swerve, this->navigation, std::make_unique<stingers::math::LinearPath>(glm::vec2(4.0, 0.0), glm::vec2(4.0, 4.0)), config).ToPtr(),
    stingers::FollowPath(this->swerve, this->navigation, std::make_unique<stingers::math::LinearPath>(glm::vec2(4.0, 4.0), glm::vec2(0.0, 4.0)), config).ToPtr(),
    stingers::FollowPath(this->swerve, this->navigation, std::make_unique<stingers::math::LinearPath>(glm::vec2(0.0, 4.0), glm::vec2(0.0, 0.0)), config).ToPtr()
  );*/

  return frc2::cmd::Sequence(frc2::cmd::Parallel(
    this->swerve.drive_command([] { return 0_mps; }, [] { return 0_mps; }, [] { return units::radians_per_second_t(0); }),
    this->turret.aim_at_command(this->navigation, [this] { return this->hub_position(); }),
    this->accelerator.run_command(1.0),
    frc2::cmd::Sequence(
      frc2::cmd::Wait(units::second_t(2)),
      this->indexer.run_command(-1.0)
    )
  ).WithTimeout(units::second_t(15)),
  frc2::cmd::Parallel(
  this->accelerator.run_command(0.0),
  this->indexer.run_command(0.0)
  ));
}
