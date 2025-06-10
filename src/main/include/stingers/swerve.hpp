#pragma once

#include <memory>
#include <vector>
#include <ctre/phoenix6/TalonFX.hpp>
#include <units/angle.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/angular_velocity.h>

namespace stingers {
namespace swerve {

class DriveMotor {
public:
	virtual void set_ground_speed_setpoint(units::velocity::meters_per_second_t) = 0;

	virtual ~DriveMotor() = 0;
};

class TurnMotor {
public:
	virtual void set_angle_setpoint_modspace(units::angle::radian_t) = 0;

	virtual ~TurnMotor() = 0;
};

class Module {
public:
	inline void set_velocity_setpoint_modspace_radial(units::angle::radian_t angle, units::velocity::meters_per_second_t speed) {
		this->turn_motor->set_angle_setpoint_modspace(angle);
		this->drive_motor->set_ground_speed_setpoint(speed);
	}
	inline void set_velocity_setpoint_modspace_cartesian(units::velocity::meters_per_second_t x, units::velocity::meters_per_second_t y) {
		units::angle::radian_t theta = units::angle::radian_t(std::atan2(y.value(), x.value()));
		units::velocity::meters_per_second_t magnitude = units::velocity::meters_per_second_t(std::sqrt(x.value()*x.value() + y.value()*y.value()));

		this->set_velocity_setpoint_modspace_radial(theta, magnitude);
	}
private:
	std::unique_ptr<DriveMotor> drive_motor;
	std::unique_ptr<TurnMotor> turn_motor;

	friend class SwerveDrive;
	units::meter_t cframe_to_cmodule_x;
	units::meter_t cframe_to_cmodule_y;
};

class SwerveDrive {
public:
	SwerveDrive(std::vector<Module> modules);

	void set_velocity_setpoint_framespace(units::velocity::meters_per_second_t x, units::velocity::meters_per_second_t y, units::angular_velocity::radians_per_second_t t);
private:
	std::vector<Module> modules;
};

}
}
