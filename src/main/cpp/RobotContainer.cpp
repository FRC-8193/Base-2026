// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc2/command/Commands.h>
#include <RobotContainer.h>

RobotContainer::RobotContainer() {
	ConfigureBindings();

	this->swerve.SetDefaultCommand(this->swerve.drive_command(
		[this] { return this->driver.GetX(); },
	));
}

void RobotContainer::ConfigureBindings() {}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
	return frc2::cmd::Print("No autonomous command configured");
}
