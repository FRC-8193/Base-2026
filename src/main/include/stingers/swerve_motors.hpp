#include <stingers/swerve.hpp>
#include <ctre/phoenix6/TalonFX.hpp>

namespace stingers::swerve::motors {

class TalonFxDriveMotor : public DriveMotor {
public:
	TalonFxDriveMotor(int id) : motor(id) {}

	void set_ground_speed_setpoint(units::velocity::meters_per_second_t speed) override;

	~TalonFxDriveMotor() override = default;
private:
	ctre::phoenix6::hardware::TalonFX motor;
};

class TalonFxTurnMotor : public TurnMotor {
public:
	TalonFxTurnMotor(int id) : motor(id) {}

	void set_angle_setpoint_modspace(units::angle::radian_t angle) override;

	~TalonFxTurnMotor() override = default;
private:
	ctre::phoenix6::hardware::TalonFX motor;
};

}
