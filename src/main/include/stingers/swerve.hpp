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

/**
* An abstract motor driving a wheel.
*/
class DriveMotor {
public:
	/**
	* Set the setpoint for the ground speed of the module.
	*/
	virtual void set_ground_speed_setpoint(units::velocity::meters_per_second_t) = 0;

	virtual ~DriveMotor() = 0;
};

/**
* An abstract motor turning a wheel.
*/
class TurnMotor {
public:
	/**
	* Set the setpoint for the angle of the wheel relative to frame forward.
	*/
	virtual void set_angle_setpoint_modspace(units::angle::radian_t) = 0;

	virtual ~TurnMotor() = 0;
};

/**
* A single module, consisting of a turnand drive motor and a frame-relative position.
*/
class Module {
public:
	/**
	* Set the setpoint for the ground speed and angle of the module relative to frame forward.
	*/
	inline void set_velocity_setpoint_modspace_radial(units::angle::radian_t angle, units::velocity::meters_per_second_t speed) {
		this->turn_motor->set_angle_setpoint_modspace(angle);
		this->drive_motor->set_ground_speed_setpoint(speed);
	}
	
	/**
	* Set the setpoint for the ground speed of the module as x and y components relative to frame forward.
	* (+y = forward +x = right)
	*/
	inline void set_velocity_setpoint_modspace_cartesian(units::velocity::meters_per_second_t x, units::velocity::meters_per_second_t y) {
		units::angle::radian_t theta = units::angle::radian_t(std::atan2(y.value(), x.value()));
		units::velocity::meters_per_second_t magnitude = units::velocity::meters_per_second_t(std::sqrt(x.value()*x.value() + y.value()*y.value()));

		this->set_velocity_setpoint_modspace_radial(theta, magnitude);
	}
private:
	std::unique_ptr<DriveMotor> drive_motor;
	std::unique_ptr<TurnMotor> turn_motor;

	// let the drive controller access the position of the module
	friend class SwerveDrive;
	units::meter_t cframe_to_cmodule_x;
	units::meter_t cframe_to_cmodule_y;
};

/**
* A full drive, a collection of modules.
*/
class SwerveDrive {
public:
	/**
	* Create a new swerve drive with the specified modules.
	*/
	SwerveDrive(std::vector<Module> modules);

	/**
	* Set the setpoint for the linear and angular speed of the frame as x and y components and an angular velocity.
	* (+y = frame forward +x = frame right +theta = clockwise)
	*/
	void set_velocity_setpoint_framespace(units::velocity::meters_per_second_t x, units::velocity::meters_per_second_t y, units::angular_velocity::radians_per_second_t t);
private:
	std::vector<Module> modules;
};

}
}
