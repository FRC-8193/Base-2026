/**
*   Swerve drive implementations for FRC #8193's FRC 2026 season chassis.
*
*   Copyright (C) 2025 Frederick Ziola, et al. (New Lothrop Robotics)
*   This program is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.

*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.

*   You should have received a copy of the GNU General Public License
*   along with this program.  If not, see <https://www.gnu.org/licenses/>.
*
*   Contact us: robotics@newlothrop.k12.mi.us
*/

#include "units/velocity.h"
#include <stingers/swerve.hpp>
#include <stingers/swerve_motors.hpp>
#include <iostream>

namespace stingers::swerve {

SwerveDrive::SwerveDrive(const Configuration& config) {
	std::vector<Module> modules;

	for (const auto& mod_conf : config.modules) {
		Module module;
		module.cframe_to_cmodule_x = mod_conf.frame_offset_x;
		module.cframe_to_cmodule_y = mod_conf.frame_offset_y;

		switch (mod_conf.drive_type) {
			case TALON_FX:
				module.drive_motor = std::make_unique<motors::TalonFxDriveMotor>(mod_conf.drive_id);
			break;
			default:
				std::cerr << "ERROR: Unknown drive motor type!" << std::endl;
				std::exit(-1);
			break;
		}

		switch (mod_conf.turn_type) {
			case TALON_FX:
				module.turn_motor = std::make_unique<motors::TalonFxTurnMotor>(mod_conf.turn_id);
			break;
			default:
				std::cerr << "ERROR: Unknown turn motor type!" << std::endl;
				std::exit(-1);
			break;
		}
	}
}

void SwerveDrive::set_velocity_setpoint_framespace(units::velocity::meters_per_second_t x, units::velocity::meters_per_second_t y, units::angular_velocity::radians_per_second_t t) {
	// we want numbers we can actually use
	double dx = x.value();
	double dy = y.value();

	double dt = t.value();

	// each module demands some CPU power
	for (auto& mod : this->modules) {
		// module position
		double mx = mod.cframe_to_cmodule_x.value();
		double my = mod.cframe_to_cmodule_y.value();
		
		// module delta position (drive component)
		double mddx = dx;
		double mddy = dy;

		// module delta position (turn component)
		// this may seem a bit confusing but it actually makes sense. when rotating we want the module to
		// move in a direction tangent to a circle centered at the frame origin passing through the module.
		// finding this direction is equivalent to rotating the vector pointing from the frame to the module by 90 degrees.
		// we use the clockwise rotation formula, as the turn speed is clockwise-positive.
		// then we scale it by the turn speed so that we can turn less or in the other direction.
		double mtdx =  my * dt;
		double mtdy = -mx * dt;

		// module delta position overall is just the sum of the components
		double mdx = mddx + mtdx;
		double mdy = mddy + mtdy;

		mod.set_velocity_setpoint_modspace_cartesian(units::velocity::meters_per_second_t(mdx), units::velocity::meters_per_second_t(mdy));
	}
}

}
