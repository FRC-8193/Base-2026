 #include <stingers/commands/toggle_roller.hpp>

ToggleRollerCommand::ToggleRollerCommand(stingers::IntakeSubsystem* intake, bool on)
  : InstantCommand([intake, on] { intake->set_roller(on); }, {intake}) {}