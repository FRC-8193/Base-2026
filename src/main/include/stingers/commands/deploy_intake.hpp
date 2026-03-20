#pragma once

#include <frc2/command/InstantCommand.h>
#include <stingers/subsystems/intake.hpp>

namespace stingers {

class DeployIntakeCommand : public frc2::InstantCommand {
public:
  DeployIntakeCommand(stingers::IntakeSubsystem* intake, bool deploy);
};

}
