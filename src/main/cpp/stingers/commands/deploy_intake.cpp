#include <stingers/commands/deploy_intake.hpp>

namespace stingers {

DeployIntakeCommand::DeployIntakeCommand(stingers::IntakeSubsystem* intake, bool deploy)
  : InstantCommand([intake, deploy] { intake->deploy(deploy); }, {intake}) {}

}
