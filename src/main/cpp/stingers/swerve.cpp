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

namespace stingers {
namespace swerve {

void SwerveDrive::set_velocity_setpoint_framespace(units::velocity::meters_per_second_t x, units::velocity::meters_per_second_t y, units::angular_velocity::radians_per_second_t t) {
	double dx = x.value();
	double dy = y.value();

	double dt = t.value();

	for (auto& mod : this->modules) {
		double mx = mod.cframe_to_cmodule_x.value();
		double my = mod.cframe_to_cmodule_y.value();

		double mddx = dx;
		double mddy = dy;
		
		double mtdx = -my * dt;
		double mtdy =  mx * dt;
	
		double mdx = mddx + mtdx;
		double mdy = mddy + mtdy;

		mod.set_velocity_setpoint_modspace_cartesian(units::velocity::meters_per_second_t(mdx), units::velocity::meters_per_second_t(mdy));
	}
}

}
}
