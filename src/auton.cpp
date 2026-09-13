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
    chassis.setPose(60, 0, 270);
    startMcl(60, 0, 270, false, false, true, false);
    MclMain.setObstacles(&autonObstacles, nullptr);

    // Get cup
    chassis.moveToPoint(19, 0, 1600, {.minSpeed=80}, true);
    pros::delay(200);
    hardResetEffector();
    pros::delay(400);
    startFrontIntake();
    startEffectorIntake();
    pros::delay(1000);

    // Score pin + cup on neutral base
    chassis.turnToPoint(47, -23.5, 500, {.forwards=false}, true);
    pros::delay(1000);
    chassis.moveToPoint(47, -23.5, 1200, {.forwards=false, .maxSpeed=80}, true);
    scoreCup();
    pros::delay(600);

    // Grab pin
    chassis.turnToHeading(260, 600, {}, true);
    pros::delay(400);
    resetEffector();
    startFrontIntake();
    startEffectorIntake();
    moveForward(6.5, 800, 90, 1, true);
    chassis.turnToPoint(58, 0, 600, {}, true);
    chassis.moveToPoint(58, 0, 1200, {}, true);
    chassis.turnToHeading(90, 600, {}, true);

    pros::delay(1000);
    RclMain.setRclPose({60.0, 0, chassis.getPose().theta});
    chassis.setPose(60.0, 0, chassis.getPose().theta);
    MclMain.set_pose(RclMain.updateBotPose(&frontL_rcl).second, RclMain.updateBotPose(&left_rcl).second, chassis.getPose().theta);

    moveForward(20, 800, 127, 50, true);
    chassis.turnToPoint(47, 23.5, 400, {.forwards=false}, true);
    chassis.moveToPoint(47, 23.5, 1500, {.forwards=false, .maxSpeed=80}, true);
    pros::delay(600);
    setEffector(HIGH_ANGLE);
    pros::delay(900);
    scorePin();
    pros::delay(600);

    // Grab pin and cup
    chassis.moveToPoint(40, 8, 1500, {.maxSpeed=80}, true);
    pros::delay(1200);
    chassis.turnToPoint(25, 23.5, 600, {.forwards=false}, true);
    setEffector(HIGH_ANGLE);
    chassis.moveToPoint(25, 23.5, 1500, {.forwards=false, .maxSpeed=45}, true);
    startEffectorIntake();
    pros::delay(1000);
    setEffector(LOW_ANGLE);
    pros::delay(600);
    setEffector(RIGHT_ANGLE);

    // Score again
    chassis.turnToPoint(47, 23.5, 400, {.forwards=false}, true);
    chassis.moveToPoint(47, 23.5, 1000, {.forwards=false, .maxSpeed=80}, true);
    pros::delay(300);
    setEffector(HIGH_ANGLE);
    pros::delay(700);
    scoreCup();
    pros::delay(600);
}