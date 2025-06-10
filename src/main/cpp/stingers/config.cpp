/**
*   Configuration options for FRC #8193's FRC 2026 season chassis.
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

#include <stingers/swerve.hpp>

const stingers::swerve::Configuration swerve_config = {
	.modules = {
		// Front left
		stingers::swerve::Configuration::Module {
			.turn_type = TALON_FX,
			.turn_id = 0,
			.drive_type = TALON_FX,
			.drive_id = 1,

			.frame_offset_x = -0.5_m,
			.frame_offset_y =  0.5_m
		},
	}
};
