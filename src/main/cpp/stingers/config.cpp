#include <stingers/swerve.hpp>

const stingers::swerve::Configuration swerve_config = {
	.modules = {
		stingers::swerve::Configuration::Module {
			.turn_type = TALON_FX,
			.turn_id = 0,
			.drive_type = TALON_FX,
			.drive_id = 1,
		},
	}
};
