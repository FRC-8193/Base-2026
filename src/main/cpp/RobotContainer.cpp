// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <RobotContainer.h>
#include <frc2/command/Commands.h>
#include <iostream>

RobotContainer::RobotContainer() : driver(0) {
  ConfigureBindings();

  this->swerve.SetDefaultCommand(this->swerve.drive_command(
      [this] { return units::meters_per_second_t(this->driver.GetX()); },
      [this] { return units::meters_per_second_t(this->driver.GetY()); },
      [this] { return units::radians_per_second_t(this->driver.GetTwist()); }));
}

void RobotContainer::ConfigureBindings() {}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
