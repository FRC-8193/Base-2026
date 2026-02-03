// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/smartdashboard/SmartDashboard.h>
#include <RobotContainer.h>
#include <frc2/command/Commands.h>
#include <iostream>

RobotContainer::RobotContainer() : navigation(swerve), driver(0) {
  ConfigureBindings();

  frc::SmartDashboard::PutData("swerve", &this->swerve);
}

void RobotContainer::ConfigureBindings() {
  this->swerve.SetDefaultCommand(this->swerve.drive_command(
      [this] { return units::meters_per_second_t(this->driver.GetRawAxis(0))*2.0; },
      [this] { return units::meters_per_second_t(this->driver.GetRawAxis(1))*2.0; },
      [this] { return units::radians_per_second_t(this->driver.GetRawAxis(2)); }));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
