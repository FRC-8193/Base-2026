#include <stingers/commands/deploy_intake.hpp>

DeployIntakeCommand::DeployIntakeCommand(stingers::IntakeSubsystem* intake, bool deploy)
  : InstantCommand([intake, deploy] { intake->deploy(deploy); }, {intake}) {}

