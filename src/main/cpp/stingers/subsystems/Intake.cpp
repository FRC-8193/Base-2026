#include "stingers/subsystems/intake.hpp"

using namespace ctre::phoenix6;

stingers::IntakeSubsystem::IntakeSubsystem() : left_intake_deploy_motor(left_intake_deploy_canid), right_intake_deploy_motor(right_intake_deploy_canid), intake_roller_motor(intake_roller_canid) {
}

void stingers::IntakeSubsystem::deploy(bool deploy) {
  const auto target = deploy ? deploy_position : retract_position;
  controls::PositionVoltage request{target}; // create a Motion Magic request, voltage output
  left_intake_deploy_motor.SetControl(request); //target position found in intake.hpp
  right_intake_deploy_motor.SetControl(request);
}
 

void stingers::IntakeSubsystem::set_roller(bool on) {
  intake_roller_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,
  on ? 1.0 : 0.0);
}   