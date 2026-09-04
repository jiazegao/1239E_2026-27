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
    chassis.calibrate();
    chassis.setPose(0, 0, 0);
	// pros::lcd::initialize();

	init_auton_selector();
	initControllerDisplay();
	initLog();	// Critical; DO NOT REMOVE
	initEffectorMacro();

	effectorRotateMotor.set_encoder_units(pros::MotorEncoderUnits::deg);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
}

void opcontrol() {
	while (true) {

		updateTankDrive();
		updateLiftMotors();

		pros::delay(20);
	}
}