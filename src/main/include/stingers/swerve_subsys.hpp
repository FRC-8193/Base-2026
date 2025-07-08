#pragma once

#include <functional>
#include <stingers/swerve.hpp>
#include <frc2/command/SubsystemBase.h>

namespace stingers::swerve {

class SwerveSubsystem : public frc2::SubsystemBase {
public:
	/**
	* Creates a new swerve subsystem using the configuration defined in `cpp/stingers/config.cpp`.
	*/
	SwerveSubsystem();

	/**
	* Sets the target speed and angular speed for the swerve drive.
	*/
	void drive_framespace(units::velocity::meters_per_second_t x, units::velocity::meters_per_second_t y, units::angular_velocity::radians_per_second_t t);

	/**
	* Creates a command that gets the target speed and angular speed for the  swerve from a lambda and sets the swerve parameters
	*/
	frc2::CommandPtr drive_command(
		std::function<units::velocity::meters_per_second_t()>,
		std::function<units::velocity::meters_per_second_t()>,
		std::function<units::angular_velocity::radians_per_second_t()>
	);

	void InitSendable(wpi::SendableBuilder& builder) override;
private:
	SwerveDrive drive;
};
}
