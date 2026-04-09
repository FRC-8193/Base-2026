// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/button/CommandJoystick.h>
#include <frc2/command/CommandPtr.h>
#include <stingers/subsystems/swerve.hpp>
#include <stingers/subsystems/navigation.hpp>
#include <stingers/subsystems/imu.hpp>
#include <stingers/subsystems/turret.hpp>
#include <stingers/subsystems/intake.hpp>
#include <stingers/subsystems/indexer.hpp>
#include <stingers/subsystems/accelerator.hpp>
#include <frc/DriverStation.h>

class RobotContainer {
public:
  RobotContainer();

  inline bool is_blue() const { return frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue; }
  inline float driver_forward_radians() const { return this->is_blue() ? 0.0f : M_PI; }
  inline glm::vec2 hub_position() const { return glm::vec2(this->is_blue() ? 4.62 : 11.91, 4.0f); }

  frc2::CommandPtr GetAutonomousCommand();

private:
  void ConfigureBindings();

  stingers::IMUSubsystem imu;
  stingers::swerve::SwerveSubsystem swerve;
  stingers::NavigationSubsystem navigation;
  stingers::TurretSubsystem turret;
  stingers::IntakeSubsystem intake;
  stingers::IndexerSubsystem indexer;
  stingers::AcceleratorSubsystem accelerator;

  frc2::CommandJoystick driver;
};
