#include <stingers/swerve_subsys.hpp>
#include <frc2/command/Commands.h>

namespace stingers::swerve {

SwerveSubsystem::SwerveSubsystem() : drive(swerve_config) {
	this->SetDefaultCommand(frc2::cmd::Run([this]() {
		this->drive_framespace(0_mps, 0_mps, 0_rad_per_s);
	}));
}

void SwerveSubsystem::drive_framespace(units::velocity::meters_per_second_t x, units::velocity::meters_per_second_t y, units::angular_velocity::radians_per_second_t t) {
	this->drive.set_velocity_setpoint_framespace(x, y, t);
}

void SwerveSubsystem::InitSendable(wpi::SendableBuilder& builder) {
	builder.AddStringProperty("TODO", []() { return "Swerve sendable"; }, [](std::string_view) {});
}
}
