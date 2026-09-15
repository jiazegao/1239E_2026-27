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

void toggle() {
    moveForward(15, 400, 127, 30, true);
    moveForward(-5, 300, 127, 30, true);
}

void blueLeft() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(60, 0, 90);
    MclMain.setObstacles(&autonObstacles, nullptr);
    startMcl(60, 0, 90, true, false, false, false);

    // Toggle
    toggle();
    toggle();

    // Get cup
    moveForward(-9, 500, 127, 1, true);
    chassis.turnToHeading(270, 800, {}, true);
    chassis.moveToPoint(15, 0, 1400, {.minSpeed=80}, true);
    pros::delay(200);
    hardResetEffector();
    pros::delay(400);
    startFrontIntake();
    startEffectorIntake();
    pros::delay(600);

    // Score pin + cup on neutral base
    chassis.turnToPoint(47, -23.5, 500, {.forwards=false}, true);
    moveForward(2, 500, 40, 1, true);
    chassis.moveToPoint(47, -23.5, 1200, {.forwards=false, .maxSpeed=90}, true);
    pros::delay(200);
    setEffector(RIGHT_ANGLE);
    pros::delay(200);
    setLift(11);
    pros::delay(600);
    scoreCup(15);
    pros::delay(1100);

    // Grab pin
    leftMotors.move(-127);
    rightMotor.move(30);
    pros::delay(500);
    leftMotors.move(0);
    rightMotors.move(0);
    moveForward(15, 2000, 50, 1, true);
    resetEffector();
    pros::delay(200);
    resetLift();
    startFrontIntake();
    startEffectorIntake();

    // Score pin
    chassis.turnToPoint(47, 23.5, 400, {.forwards=false}, true);
    chassis.moveToPoint(47, 23.5, 1500, {.forwards=false, .maxSpeed=100}, true);
    pros::delay(800);
    setEffector(RIGHT_ANGLE);
    pros::delay(900);
    scorePin();
    pros::delay(600);

    // Grab pin and cup
    chassis.moveToPoint(40, 8, 1000, {.maxSpeed=80}, true);
    chassis.turnToPoint(25, 23.5, 500, {.forwards=false}, true);
    setEffector(HIGH_ANGLE);
    chassis.moveToPoint(25, 23.5, 1500, {.forwards=false, .maxSpeed=45}, true);
    startEffectorIntake();
    pros::delay(1000);
    setEffector(LOW_ANGLE);
    pros::delay(600);
    setEffector(RIGHT_ANGLE);

    // Score again
    chassis.turnToPoint(47, 23.5, 400, {.forwards=false}, true);
    setEffector(RIGHT_ANGLE);
    chassis.moveToPoint(47, 23.5, 1000, {.forwards=false, .maxSpeed=80}, true);
    setLift(11);
    pros::delay(600);
    scoreCup(15);
}