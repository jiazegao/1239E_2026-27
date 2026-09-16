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
    moveForward(15, 400, 127, 100, true);
    moveForward(-5, 300, 127, 60, true);
}

void blueLeft() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(60, 0, 90);
    MclMain.setObstacles(&autonObstacles, nullptr);
    startMcl(60, 0, 90, true, false, false, false);

    // Toggle
    moveForward(15, 300, 127, 100, true);
    moveForward(-5, 300, 127, 60, true);
    moveForward(15, 500, 127, 100, true);
    hardResetEffector();

    // Get cup
    moveForward(-14, 800, 127, 1, true);
    chassis.turnToHeading(270, 800, {}, true);
    chassis.moveToPoint(13, 0, 1000, {}, true);
    startFrontIntake();
    startEffectorIntake();

    // Score pin + cup on neutral base
    chassis.turnToPoint(45.5, -24.0, 500, {.forwards=false}, true);
    chassis.moveToPoint(45.5, -24.0, 1300, {.forwards=false, .maxSpeed=75}, true);
    pros::delay(500);
    setEffector(RIGHT_ANGLE);
    pros::delay(200);
    setLift(10);
    pros::delay(700);
    leftMotors.move(-80);
    rightMotors.move(-20);
    scoreCup(2, 15);
    pros::delay(300);
    leftMotors.move(0);
    rightMotors.move(0);
    pros::delay(500);

    // Grab pin
    leftMotors.move(40);
    rightMotors.move(40);
    pros::delay(300);
    resetEffector();
    resetLift();
    startFrontIntake();
    startEffectorIntake();
    pros::delay(200);
    leftMotors.move(20);
    rightMotors.move(20);
    pros::delay(700);

    // Score pin
    chassis.turnToHeading(240, 500, {}, true);
    chassis.moveToPoint(49, 20.5, 1100, {.forwards=false, .maxSpeed=90}, true);
    startFrontIntake();
    startEffectorIntake();
    pros::delay(1100);
    reverseFrontIntake();
    setEffector(RIGHT_ANGLE);
    leftMotors.move(-80);
    rightMotors.move(-40);
    pros::delay(300);
    leftMotors.move(0);
    rightMotors.move(0);
    scorePin();
    pros::delay(400);
    stopFrontIntake();

    // Grab pin and cup
    chassis.moveToPoint(40, 8, 1500, {.maxSpeed=70}, true);
    chassis.turnToPoint(25, 23.5, 500, {.forwards=false}, true);
    setEffector(HIGH_ANGLE);
    startEffectorIntake();
    chassis.moveToPoint(25, 23.5, 1100, {.forwards=false, .maxSpeed=50}, false);
    leftMotors.move(-20);
    rightMotors.move(-20);
    setEffector(IDLE_ANGLE);
    pros::delay(600);

    // Score again
    chassis.turnToPoint(47, 23.5, 400, {.forwards=false}, true);
    setEffector(RIGHT_ANGLE);
    chassis.moveToPoint(47, 23.5, 1000, {.forwards=false, .maxSpeed=80}, true);
    setLift(10);
    pros::delay(500);
    scoreCup(2, 2);
}