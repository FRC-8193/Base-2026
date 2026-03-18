#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>

namespace stingers {

// set canID's in config.cpp file
const extern int left_intake_deploy_canid;
const extern int right_intake_deploy_canid;
const extern int intake_roller_canid;

class IntakeSubsystem : public frc2::SubsystemBase {
public:
  IntakeSubsystem();

  void deploy(bool deploy);
  void set_roller(bool on);


  static constexpr units::turn_t deploy_position = 10_tr;
  static constexpr units::turn_t retract_position = 0_tr;

  ctre::phoenix6::hardware::TalonFX left_intake_deploy_motor;
  ctre::phoenix6::hardware::TalonFX right_intake_deploy_motor;
  ctre::phoenix::motorcontrol::can::TalonSRX intake_roller_motor;
};
}