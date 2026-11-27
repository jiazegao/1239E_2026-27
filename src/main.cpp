#include "main.h"
#include "custom/RclTracking.hpp"
#include "custom/Tracking_Util.hpp"
#include "custom/configs.hpp"
#include "custom/auton.hpp"

#include "custom/util_funcs.hpp"
#include "custom/auton_selector.hpp" // IWYU pragma: keep
#include "liblvgl/llemu.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/motors.h"

#include "custom/lift_control.hpp"

void initialize() {
	// pros::lcd::initialize();
    chassis.calibrate();
    chassis.setPose(0, 0, 0);

	init_auton_selector();
	initControllerDisplay();
	initLog();	// Critical; DO NOT REMOVE
	initLiftPID();
	//initBrainDisplay();

	// Override gearset for 5.5 motors
	leftMotors.set_gearing(pros::MotorGears::green, 0);
	rightMotors.set_gearing(pros::MotorGears::green, 0);

	leftMotors.set_encoder_units_all(pros::MotorEncoderUnits::degrees);
	rightMotors.set_encoder_units_all(pros::MotorEncoderUnits::degrees);

	liftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
	liftMotors.set_encoder_units_all(pros::MotorEncoderUnits::degrees);
	liftMotors.set_zero_position_all(0.0);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	enableLiftPID = true;

	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
	// hardResetClaw();
	clawGrippersMotor.set_zero_position(13.0);
	// runAuton();
	left5pin();
}

void opcontrol() {

	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
	enableLiftPID = false;
	startBrainFBDisplay();

	hardResetClaw();
	resetLift();

	while (true) {

		updateTankDrive();
		updateLiftMotors();

		pros::delay(20);
	}
}