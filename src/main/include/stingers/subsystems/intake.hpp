#pragma once

#include <frc2/command/SubsystemBase.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>

namespace stingers {

// set canID's in config.cpp file
const extern int intake_deploy_canid;
const extern int intake_roller_canid;

class IntakeSubsystem : public frc2::SubsystemBase {
public:
  IntakeSubsystem();

  void deploy(bool deploy);
  void set_roller(bool on);

private:
  static constexpr float deploy_position = 6600;
  static constexpr float retract_position = 400;

  ctre::phoenix::motorcontrol::can::TalonSRX intake_deploy_motor;
  ctre::phoenix::motorcontrol::can::VictorSPX intake_roller_motor;
};
}
