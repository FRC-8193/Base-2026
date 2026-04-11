#include "stingers/subsystems/intake.hpp"

stingers::IntakeSubsystem::IntakeSubsystem() : intake_deploy_motor(intake_deploy_canid), intake_roller_motor(intake_roller_canid) {
  //this->intake_deploy_motor.SetPosition(0);
}

void stingers::IntakeSubsystem::deploy(bool deploy) {
  const auto target = deploy ? deploy_position : retract_position;
  intake_deploy_motor.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::Position, (float)target);
}
 

void stingers::IntakeSubsystem::set_roller(bool on) {
  intake_roller_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,
  on ? -1.0 : 0.0);
}   
