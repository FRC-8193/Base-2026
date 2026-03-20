#pragma once

#include <frc2/command/InstantCommand.h>
#include <stingers/subsystems/intake.hpp>

namespace stingers {

class ToggleRollerCommand : public frc2::InstantCommand {
public:
  ToggleRollerCommand(stingers::IntakeSubsystem* intake, bool on);
 };   

}
