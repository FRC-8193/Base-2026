// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Joystick.h>
#include <frc2/command/CommandPtr.h>
#include <stingers/swerve_subsys.hpp>

class RobotContainer {
public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

private:
  void ConfigureBindings();

  stingers::swerve::SwerveSubsystem swerve;

  frc::Joystick driver = frc::Joystick{0};
};
