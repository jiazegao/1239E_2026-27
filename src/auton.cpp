#include "custom/auton.hpp"
#include "custom/MclTracking.hpp"
#include "custom/RclTracking.hpp"
#include "custom/configs.hpp"
#include "custom/util_funcs.hpp"
#include "custom/lift_control.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pose.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include <cmath>

void blueLeft() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(64, 0, 270);
    MclMain.setObstacles(&autonObstacles, nullptr);
    startMcl(64, 0, 270, false, false, true, true);

    // Toggle then get cup
    effectorToggle();
    pros::delay(400);
    chassis.moveToPoint(24, 0, 1000, {}, true);
    pros::delay(300);
    hardResetEffector();
    startFrontIntake();

    // Score pin + cup on neutral base
    chassis.turnToPoint(47, -23.5, 500, {.forwards=false}, true);
    chassis.moveToPoint(47, -23.5, 1300, {.forwards=false, .maxSpeed=80}, true);
    pros::delay(400);
    setEffector(HIGH_ANGLE);
    pros::delay(900);
    scoreObject();
    pros::delay(600);

    // Grab pin
    chassis.swingToHeading(250, lemlib::DriveSide::LEFT, 800, {.maxSpeed=80}, true);
    pros::delay(400);
    resetEffector();
    chassis.turnToPoint(47, 23.5, 400, {.forwards=false}, true);
    chassis.moveToPoint(47, 23.5, 1500, {.forwards=false, .maxSpeed=80}, true);
    pros::delay(600);
    setEffector(HIGH_ANGLE);
    pros::delay(900);
    scoreObject();
    pros::delay(600);

    // Grab pin and cup
    moveForward(10, 800, 127, 1, true);
    chassis.turnToPoint(23.5, 23.5, 600, {.forwards=false}, true);
    moveForward(-20, 2000, 60, 1, false);
    startEffectorIntake();
    setEffector(RIGHT_ANGLE);
    pros::delay(600);

    // Score again
    chassis.turnToPoint(47, 23.5, 400, {.forwards=false}, true);
    chassis.moveToPoint(47, 23.5, 1000, {.forwards=false, .maxSpeed=80}, true);
    pros::delay(300);
    setEffector(HIGH_ANGLE);
    pros::delay(700);
    scoreObject();
    pros::delay(600);
}