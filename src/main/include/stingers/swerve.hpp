/**
*   Swerve drive declarations for FRC #8193's FRC 2026 season chassis.
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

#pragma once

#include <memory>
#include <vector>
#include <ctre/phoenix6/TalonFX.hpp>
#include <units/angle.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/angular_velocity.h>

enum MotorType {
	TALON_FX
};

namespace stingers {
namespace swerve {

struct Configuration {
	struct Module {
		/**
		* The type of the motor controller turning this wheel.
		*/
		MotorType turn_type;
		/**
		* The CAN id of the motor controller turning this wheel.
		*/
		int turn_id;

		/**
		* The type of the motor controller driving this wheel.
		*/
		MotorType drive_type;
		/**
		* The CAN id of the motor controller driving this wheel.
		*/
		int drive_id;

		/**
		* The distance in meters from the center of the frame to the center of the module, over the X (right) dimension.
		*/
		units::meter_t frame_offset_x;
		/**
		* The distance in meters from the center of the frame to the center of the modules, over the Y (forward) dimension.
		*/
		units::meter_t frame_offset_y;
	};

	std::vector<Module> modules;
};

const Configuration swerve_config;

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
