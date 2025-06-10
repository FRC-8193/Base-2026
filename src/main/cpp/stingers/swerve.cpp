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
