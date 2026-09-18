#include "custom/auton.hpp"
#include "custom/MclTracking.hpp"
#include "custom/RclTracking.hpp"
#include "custom/configs.hpp"
#include "custom/util_funcs.hpp"
#include "custom/lift_control.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pose.hpp"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include <cmath>

void toggle() {
    moveForward(15, 400, 127, 100, true);
    moveForward(-5, 300, 127, 60, true);
}

void left40() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(60, 0, 90);
    MclMain.setObstacles(&leftAutonObstacles, nullptr);
    startMcl(60, 0, 90, true, false, false, false);

    // Toggle
    moveForward(15, 300, 127, 100, true);
    moveForward(-5, 300, 127, 60, true);
    moveForward(15, 500, 127, 100, true);
    hardResetEffector();

    // Get cup
    moveForward(-10, 800, 127, 1, true);
    chassis.turnToHeading(270, 800, {.maxSpeed=100}, true);
    chassis.moveToPoint(14, 0, 200, {.minSpeed = 90}, true);
    chassis.moveToPoint(14, 0, 1000, {.maxSpeed = 60}, true);
    startFrontIntake();
    startEffectorIntake();

    // Score pin + cup on neutral base
    chassis.turnToPoint(44, -24, 500, {.forwards=false}, true);
    chassis.moveToPoint(44, -24, 1400, {.forwards=false, .maxSpeed=75}, true);
    pros::delay(500);
    setEffector(EFFECTOR_ANGLES[RIGHT_ANGLE]+3.0);
    pros::delay(300);
    setLift(8);
    pros::delay(700);
    leftMotors.move(-100);
    rightMotors.move(-40);
    scoreCup(2, 15);
    pros::delay(500);
    leftMotors.move(0);
    rightMotors.move(0);
    pros::delay(300);

    // Grab pin
    leftMotors.move(40);
    rightMotors.move(60);
    pros::delay(200);
    resetEffector();
    resetLift();
    startFrontIntake();
    startEffectorIntake();
    pros::delay(400);
    leftMotors.move(35);
    rightMotors.move(10);
    pros::delay(600);

    // Score pin
    chassis.moveToPoint(47, 10, 900, {.forwards=false}, true);
    startFrontIntake();
    startEffectorIntake();
    chassis.turnToHeading(180, 200, {}, true);
    chassis.moveToPoint(47, 23.5, 800, {.forwards=false, .maxSpeed=60}, true);
    pros::delay(200);
    reverseFrontIntake();
    setEffector(EFFECTOR_ANGLES[RIGHT_ANGLE]+4.0);
    pros::delay(600);
    stopEffectorIntake();
    leftMotors.move(-50);
    rightMotors.move(-50);
    scorePin();
    
    pros::delay(400);
    stopFrontIntake();

    // Grab pin and cup
    chassis.moveToPoint(40, 8, 1300, {.maxSpeed=60}, true);
    chassis.turnToPoint(25, 17, 500, {.forwards=false}, true);
    setEffector(HIGH_ANGLE);
    startEffectorIntake();

    chassis.moveToPoint(24, 24, 800, {.forwards=false, .maxSpeed=65}, false);
    
    // pros::delay(50);
    // leftMotors.move(-20);
    // rightMotors.move(-20);

    setEffector(LOW_ANGLE);
    pros::delay(150);

    // Score again
    chassis.turnToPoint(47, 23.5, 400, {.forwards=false}, true);
    setEffector(RIGHT_ANGLE);
    chassis.moveToPoint(47, 23.5, 1000, {.forwards=false, .maxSpeed=80}, true);
    setLift(8);
    pros::delay(500);
    scoreCup(2, 2);
}

void right2_1(){
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-60, 0, -90);
    MclMain.setObstacles(&rightAutonObstacles, nullptr);
    startMcl(-60, 0, -90, true, false, false, false);

    // Toggle
    moveForward(15, 300, 127, 100, true);
    moveForward(-5, 300, 127, 60, true);
    moveForward(15, 500, 127, 100, true);
    hardResetEffector();

    // Get cup
    moveForward(-10, 800, 127, 1, true);
    chassis.turnToHeading(-270, 800, {.maxSpeed=100}, true);
    chassis.moveToPoint(-14, 0, 200, {.minSpeed = 100}, true);
    chassis.moveToPoint(-14, 0, 1000, {.maxSpeed = 60}, true);
    startFrontIntake();
    startEffectorIntake();

    // Score pin + cup on neutral base
    chassis.turnToPoint(-48, -22, 500, {.forwards=false}, true);
    chassis.moveToPoint(-48, -22, 1400, {.forwards=false, .maxSpeed=75}, true);
    pros::delay(500);
    setEffector(EFFECTOR_ANGLES[RIGHT_ANGLE]+3.0);
    pros::delay(150);
    setLift(10);
    pros::delay(850);
    rightMotors.move(-100);
    leftMotors.move(-40);
    scoreCup(2, 15);
    pros::delay(500);
    rightMotors.move(0);
    leftMotors.move(0);
    pros::delay(400);

   // Grab pin
    chassis.moveToPoint(-26, -24, 900);
    // rightMotors.move(0);
    // leftMotors.move(80);
    pros::delay(300);
    resetEffector();
    resetLift();
    startFrontIntake();
    startEffectorIntake();
    pros::delay(200);
    // rightMotors.move(65);
    // leftMotors.move(45);
    pros::delay(400);

    // Score pin
    chassis.moveToPoint(-46.5, 10, 1000, {.forwards=false}, true);
    startFrontIntake();
    startEffectorIntake();
    chassis.turnToHeading(-180, 200, {}, true);
    chassis.moveToPoint(-46.5, 23.5, 800, {.forwards=false, .maxSpeed=60}, true);
    pros::delay(200);
    reverseFrontIntake();
    setEffector(EFFECTOR_ANGLES[RIGHT_ANGLE]+4.0);
    pros::delay(650);
    stopEffectorIntake();
    rightMotors.move(-50);
    leftMotors.move(-50);
    scorePin();

    pros::delay(200);
    stopFrontIntake();

    // Grab pin and cup
    chassis.moveToPoint(-40, 8, 1300, {.maxSpeed=60}, true);
    chassis.turnToPoint(-23, 15, 500, {.forwards=false}, true);
    setEffector(HIGH_ANGLE);
    startEffectorIntake();

    chassis.moveToPoint(-22, 26, 900, {.forwards=false, .maxSpeed=65}, false);

    // pros::delay(50);
    // rightMotors.move(-20);
    // leftMotors.move(-20);

    setEffector(LOW_ANGLE);
    pros::delay(300);
   chassis.turnToPoint(-58, 24, 600, {.forwards=false}, true);
    setEffector(RIGHT_ANGLE);
    chassis.moveToPoint(-45, 24, 2000, {.forwards=false, .maxSpeed=80}, true);
    setLift(5);
}

void right40() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-60, 0, -90);
    MclMain.setObstacles(&rightAutonObstacles, nullptr);
    startMcl(-60, 0, -90, true, false, false, false);

    // Toggle
    moveForward(15, 300, 127, 100, true);
    moveForward(-5, 300, 127, 60, true);
    moveForward(15, 500, 127, 100, true);
    hardResetEffector();

    // Get cup
    moveForward(-10, 800, 127, 1, true);
    chassis.turnToHeading(-270, 800, {.maxSpeed=100}, true);
    chassis.moveToPoint(-14, 0, 200, {.minSpeed = 100}, true);
    chassis.moveToPoint(-14, -2, 1000, {.maxSpeed = 60}, true);
    startFrontIntake();
    startEffectorIntake();

    // Score pin + cup on neutral base
    chassis.turnToPoint(-48, -22, 500, {.forwards=false}, true);
    chassis.moveToPoint(-48, -22, 1400, {.forwards=false, .maxSpeed=75}, true);
    pros::delay(500);
    setEffector(EFFECTOR_ANGLES[RIGHT_ANGLE]+3.0);
    pros::delay(150);
    setLift(10);
    pros::delay(850);
    rightMotors.move(-100);
    leftMotors.move(-40);
    scoreCup(2, 15);
    pros::delay(500);
    rightMotors.move(0);
    leftMotors.move(0);
    pros::delay(300);

   // Grab pin
    //chassis.swingToPoint(-26, -20, lemlib::DriveSide::RIGHT, 800);
    rightMotors.move(60);
    leftMotors.move(60);
    pros::delay(200);
    resetEffector();
    resetLift();
    startFrontIntake();
    startEffectorIntake();
    pros::delay(200);
    // rightMotors.move(65);
    // leftMotors.move(45);
    pros::delay(400);

    // Score pin
    chassis.moveToPoint(-46.5, 10, 1000, {.forwards=false}, true);
    startFrontIntake();
    startEffectorIntake();
    chassis.turnToHeading(-180, 200, {}, true);
    chassis.moveToPoint(-46.5, 23.5, 800, {.forwards=false, .maxSpeed=60}, true);
    pros::delay(200);
    reverseFrontIntake();
    setEffector(EFFECTOR_ANGLES[RIGHT_ANGLE]+4.0);
    pros::delay(650);
    stopEffectorIntake();
    rightMotors.move(-50);
    leftMotors.move(-50);
    scorePin();

    pros::delay(400);
    stopFrontIntake();

    // Grab pin and cup
    chassis.moveToPoint(-40, 8, 1300, {.maxSpeed=60}, true);
    chassis.turnToPoint(-23, 15, 500, {.forwards=false}, true);
    setEffector(HIGH_ANGLE);
    startEffectorIntake();

    chassis.moveToPoint(-24, 24, 1000, {.forwards=false, .maxSpeed=65}, false);

    // pros::delay(50);
    // rightMotors.move(-20);
    // leftMotors.move(-20);

    setEffector(LOW_ANGLE);
    pros::delay(300);

    // Score again
    chassis.turnToPoint(-47, 24, 600, {.forwards=false}, true);
    setEffector(RIGHT_ANGLE);
    chassis.moveToPoint(-47, 24, 2000, {.forwards=false, .maxSpeed=80}, true);
    setLift(5);
    pros::delay(500);
    scoreCup(2, 2);
}
void left30() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(60, 0, 90);
    MclMain.setObstacles(&leftAutonObstacles, nullptr);
    startMcl(60, 0, 90, true, false, false, false);

    // Toggle
    moveForward(15, 300, 127, 100, true);
    moveForward(-5, 300, 127, 60, true);
    moveForward(15, 500, 127, 100, true);
    hardResetEffector();

    // Get cup
    moveForward(-18, 1000, 127, 1, true);
    chassis.turnToHeading(270, 800, {.maxSpeed=100}, true);
    chassis.moveToPoint(14, 0, 1000, {}, true);
    startFrontIntake();
    startEffectorIntake();

    // Score pin + cup on neutral base
    chassis.turnToPoint(47.5, -23.5, 500, {.forwards=false}, true);
    chassis.moveToPoint(47.5, -23.5, 1600, {.forwards=false, .maxSpeed=75}, true);
    pros::delay(500);
    setEffector(RIGHT_ANGLE);
    pros::delay(300);
    setLift(12);
    pros::delay(600);
    scoreCup(2, 15);
    pros::delay(800);

    // Grab pin
    chassis.moveToPoint(40, 8, 1200, {}, true);
    pros::delay(400);
    resetEffector();
    resetLift();
    startFrontIntake();
    startEffectorIntake();
    chassis.moveToPoint(40, 8, 800, {.maxSpeed=40}, true);
    chassis.turnToPoint(27, 22.5, 500, {.forwards=false}, true);
    setEffector(HIGH_ANGLE);
    startEffectorIntake();
    chassis.moveToPoint(27, 22.5, 1400, {.forwards=false, .maxSpeed=40}, false);
    leftMotors.move(-20);
    rightMotors.move(-20);
    setEffector(IDLE_ANGLE);
    pros::delay(650);
    setEffector(RIGHT_ANGLE);
    pros::delay(300);

    // Score again
    chassis.turnToPoint(47, -24.5, 700, {.forwards=false}, true);
    chassis.moveToPoint(47, -24.5, 2000, {.forwards=false, .maxSpeed=80}, true);
    setLift(20);
    pros::delay(2000);
    scoreCup(4, 4);
}
void right30() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-60, 0, -90);
    MclMain.setObstacles(&rightAutonObstacles, nullptr);
    startMcl(-60, 0, -90, true, false, false, false);

    // Toggle
    moveForward(15, 300, 127, 100, true);
    moveForward(-5, 300, 127, 60, true);
    moveForward(15, 500, 127, 100, true);
    hardResetEffector();

    // Get cup
    moveForward(-18, 1000, 127, 1, true);
    chassis.turnToHeading(-270, 800, {.maxSpeed=100}, true);
    chassis.moveToPoint(-14, 0, 1000, {}, true);
    startFrontIntake();
    startEffectorIntake();

    // Score pin + cup on neutral base
    chassis.turnToPoint(-47.5, -23.5, 500, {.forwards=false}, true);
    chassis.moveToPoint(-47.5, -23.5, 1600, {.forwards=false, .maxSpeed=75}, true);
    pros::delay(500);
    setEffector(RIGHT_ANGLE);
    pros::delay(300);
    setLift(12);
    pros::delay(600);
    scoreCup(2, 15);
    pros::delay(800);

    // Grab pin
    chassis.moveToPoint(-40, 8, 1200, {}, true);
    pros::delay(400);
    resetEffector();
    resetLift();
    startFrontIntake();
    startEffectorIntake();
    chassis.moveToPoint(-40, 8, 800, {.maxSpeed=40}, true);
    chassis.turnToPoint(-27, 22.5, 500, {.forwards=false}, true);
    setEffector(HIGH_ANGLE);
    startEffectorIntake();
    chassis.moveToPoint(-27, 22.5, 1400, {.forwards=false, .maxSpeed=40}, false);
    leftMotors.move(-20);
    rightMotors.move(-20);
    setEffector(IDLE_ANGLE);
    pros::delay(650);
    setEffector(RIGHT_ANGLE);
    pros::delay(300);

    // Score again
    chassis.turnToPoint(-47, -24.5, 700, {.forwards=false}, true);
    chassis.moveToPoint(-47, -24.5, 2000, {.forwards=false, .maxSpeed=80}, true);
    setLift(20);
    pros::delay(2000);
    scoreCup(4, 4);
}

void left3_1(){
    // Count concurrent motion toward a mechanism's time allowance.
    const auto waitRemaining = [](std::uint32_t started, std::uint32_t duration) {
        const auto elapsed = pros::millis() - started;
        if (elapsed < duration) pros::delay(duration - elapsed);
    };

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(60, 0, 90);
    MclMain.setObstacles(&leftAutonObstacles, nullptr);
    startMcl(60, 0, 90, true, false, false, false);
    // Get toggle 
    moveForward(15, 300, 127, 100, true);
    moveForward(-5, 300, 127, 60, true);
    moveForward(15, 500, 127, 100, true);
    hardResetEffector();
    const auto resetStarted = pros::millis();
    // pros::delay(100);
    // Score on alliance base
    chassis.turnToPoint(47, 0, 500, {.forwards=false}, false);
    chassis.moveToPoint(47, 0, 1000, {.forwards=false, .maxSpeed=80, .earlyExitRange = 2}, true);
    chassis.turnToPoint(47, 23.5, 500, {.forwards=false}, false);
    // Homing takes 1200 ms; include one task tick before commanding the effector.
    waitRemaining(resetStarted, 1220);
    reverseFrontIntake();
    setEffector(EFFECTOR_ANGLES[RIGHT_ANGLE]+5);
    const auto openingEffectorStarted = pros::millis();
    chassis.moveToPoint(47, 23.5, 1000, {.forwards=false, .maxSpeed=80}, false);
    waitRemaining(openingEffectorStarted, 800);
    leftMotors.move(-65);
    rightMotors.move(-65);
    scorePin();
    pros::delay(300);
    stopFrontIntake();
    // setEffector(RIGHT_ANGLE);
     
    // Grab pin and cup
    chassis.moveToPoint(40, 8, 1300, {.maxSpeed=60}, true);
    chassis.turnToPoint(22, 22, 500, {.forwards=false}, true);
    setEffector(HIGH_ANGLE);
    startEffectorIntake();
    chassis.moveToPoint(22, 22, 1200, {.forwards=false, .maxSpeed=40}, false);
    leftMotors.move(-20);
    rightMotors.move(-20);
    setEffector(LOW_ANGLE);
    // Stop creeping deeper while the effector finishes grabbing.
    leftMotors.move(0);
    rightMotors.move(0);
    pros::delay(300);

    // Lift the effector clear before starting the goal turn.
    leftMotors.move(0);
    rightMotors.move(0);
    setEffector(RIGHT_ANGLE);
    pros::delay(200);
    setLift(8);
    const auto lift10Started = pros::millis();
    chassis.turnToPoint(47, 23.5, 400, {.forwards=false}, true);
    chassis.moveToPoint(47, 23.5, 1000, {.forwards=false, .maxSpeed=80}, true);
    // Give the goal approach its full travel allowance before lowering.
    pros::delay(1000);
    waitRemaining(lift10Started, 800);
    scoreCup(2, 10);
    // Allow time to release the cup and lift clear before leaving.
    pros::delay(1000);
  // Go to 2nd stack
    
    moveForward(15, 1000, 127, 100, false);
    setLift(0);
    chassis.turnToPoint(47, 45, 500, {.forwards=false}, true);
    setEffector(HIGH_ANGLE);
    startEffectorIntake();
    chassis.moveToPoint(47, 45, 1400, {.forwards=false, .maxSpeed=60, .minSpeed=30}, false);
    // Carry speed into the pickup and reach the pin before lowering.
    leftMotors.move(-30);
    rightMotors.move(-30);
    pros::delay(400);
    leftMotors.move(-20);
    rightMotors.move(-20);
    setEffector(IDLE_ANGLE);
    pros::delay(400);

    // Lift the effector clear before starting the goal turn.
    leftMotors.move(0);
    rightMotors.move(0);
    setEffector(RIGHT_ANGLE);
    pros::delay(200);
    setLift(15);
    const auto lift15Started = pros::millis();
    chassis.turnToPoint(47, 23.5, 400, {.forwards=false}, true);
    chassis.moveToPoint(47, 23.5, 1000, {.forwards=false, .maxSpeed=80}, true);
    // Give the goal approach its full travel allowance before lowering.
    pros::delay(1000);
    waitRemaining(lift15Started, 800);
    scoreCup(3, 20);
    pros::delay(1000);

}
void right3_1(){
    // Count concurrent motion toward a mechanism's time allowance.
    const auto waitRemaining = [](std::uint32_t started, std::uint32_t duration) {
        const auto elapsed = pros::millis() - started;
        if (elapsed < duration) pros::delay(duration - elapsed);
    };

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-60, 0, -90);
    MclMain.setObstacles(&rightAutonObstacles, nullptr);
    startMcl(-60, 0, -90, true, false, false, false);
    // Get toggle 
    moveForward(15, 300, 127, 100, true);
    moveForward(-5, 300, 127, 60, true);
    moveForward(15, 500, 127, 100, true);
    hardResetEffector();
    const auto resetStarted = pros::millis();
    // pros::delay(100);
    // Score on alliance base
    chassis.turnToPoint(-47, 0, 500, {.forwards=false}, false);
    chassis.moveToPoint(-47, 0, 1000, {.forwards=false, .maxSpeed=80}, true);
    chassis.turnToPoint(-47, 23.5, 500, {.forwards=false}, false);
    // Homing takes 1200 ms; include one task tick before commanding the effector.
    waitRemaining(resetStarted, 1220);
    reverseFrontIntake();
    setEffector(EFFECTOR_ANGLES[RIGHT_ANGLE]+4.5);
    const auto openingEffectorStarted = pros::millis();
    chassis.moveToPoint(-47, 23.5, 1000, {.forwards=false, .maxSpeed=80}, true);
    waitRemaining(openingEffectorStarted, 800);
    leftMotors.move(-65);
    rightMotors.move(-65);
    scorePin();
    pros::delay(400);
    stopFrontIntake();
    // setEffector(RIGHT_ANGLE);
     // Grab pin and cup
    chassis.moveToPoint(-40, 8, 1300, {.maxSpeed=60}, true);
    chassis.turnToPoint(-22, 22, 500, {.forwards=false}, true);
    setEffector(HIGH_ANGLE);
    startEffectorIntake();
    chassis.moveToPoint(-22, 22, 1200, {.forwards=false, .maxSpeed=40}, false);
    leftMotors.move(-20);
    rightMotors.move(-20);
    setEffector(IDLE_ANGLE);
    pros::delay(200);
    // Stop creeping deeper while the effector finishes grabbing.
    leftMotors.move(0);
    rightMotors.move(0);
    pros::delay(600);

    // Lift the effector clear before starting the goal turn.
    leftMotors.move(0);
    rightMotors.move(0);
    setEffector(RIGHT_ANGLE);
    pros::delay(500);
    setLift(10);
    const auto lift10Started = pros::millis();
    chassis.turnToPoint(-47, 23.5, 400, {.forwards=false}, true);
    chassis.moveToPoint(-47, 23.5, 1000, {.forwards=false, .maxSpeed=80}, true);
    waitRemaining(lift10Started, 800);
    scoreCup(2, 10);
    // Allow time to release the cup and lift clear before leaving.
    pros::delay(1000);
  // Go to 2nd stack
    
    moveForward(15, 1000, 127, 100, false);
    setLift(0);
    chassis.turnToPoint(-47, 45, 500, {.forwards=false}, true);
    setEffector(HIGH_ANGLE);
    startEffectorIntake();
    chassis.moveToPoint(-47, 45, 1400, {.forwards=false, .maxSpeed=60, .minSpeed=30}, false);
    // Carry speed into the pickup and reach the pin before lowering.
    leftMotors.move(-30);
    rightMotors.move(-30);
    pros::delay(400);
    leftMotors.move(-20);
    rightMotors.move(-20);
    setEffector(IDLE_ANGLE);
    pros::delay(400);

    // Lift the effector clear before starting the goal turn.
    leftMotors.move(0);
    rightMotors.move(0);
    setEffector(RIGHT_ANGLE);
    pros::delay(500);
    setLift(15);
    const auto lift15Started = pros::millis();
    chassis.turnToPoint(-47, 23.5, 400, {.forwards=false}, true);
    chassis.moveToPoint(-47, 23.5, 1000, {.forwards=false, .maxSpeed=80}, true);
    waitRemaining(lift15Started, 500);
    scoreCup(3, 20);
    pros::delay(1000);

}

void skills() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(60, 0, 90);
    startMcl(60, 0, 90, true, false, false, false);
    // Get toggle 
    moveForward(15, 300, 127, 100, true);
    moveForward(-5, 300, 127, 60, true);
    moveForward(15, 500, 127, 100, true);
    hardResetEffector();
    pros::delay(100);
    // Score on alliance base
    chassis.turnToPoint(47, 0, 500, {.forwards=false}, true);
    chassis.moveToPoint(47, 0, 1000, {.forwards=false, .maxSpeed=80}, true);
    chassis.turnToPoint(47, 23.5, 500, {.forwards=false}, true);
    chassis.moveToPoint(47, 23.5, 1000, {.forwards=false, .maxSpeed=80}, true);
    pros::delay(200);
    reverseFrontIntake();
    setEffector(EFFECTOR_ANGLES[RIGHT_ANGLE]+5.5);
    pros::delay(1000);
    leftMotors.move(-55);
    rightMotors.move(-55);
    scorePin();
    pros::delay(400);
    stopFrontIntake();
    // setEffector(RIGHT_ANGLE);
     // Grab pin and cup
    chassis.moveToPoint(40, 8, 1300, {.maxSpeed=60}, true);
    chassis.turnToPoint(23.5, 23.5, 500, {.forwards=false}, true);
    setEffector(HIGH_ANGLE);
    startEffectorIntake();
    chassis.moveToPoint(23.5, 23.5, 1200, {.forwards=false, .maxSpeed=40}, false);
    leftMotors.move(-20);
    rightMotors.move(-20);
    setEffector(IDLE_ANGLE);
    pros::delay(200);
    // Stop creeping deeper while the effector finishes grabbing.
    leftMotors.move(0);
    rightMotors.move(0);
    pros::delay(600);

    // Lift the effector clear before starting the goal turn.
    leftMotors.move(0);
    rightMotors.move(0);
    setEffector(RIGHT_ANGLE);
    pros::delay(500);
    chassis.turnToPoint(47, 23.5, 400, {.forwards=false}, true);
    chassis.moveToPoint(47, 23.5, 1000, {.forwards=false, .maxSpeed=80}, true);
    setLift(10);
    pros::delay(500);
    scoreCup(2, 10);
    // Allow time to release the cup and lift clear before leaving.
    pros::delay(1000);
  // Go to 2nd stack
    
    moveForward(15, 1000, 127, 100, false);
    setLift(0);
    chassis.turnToPoint(47, 45, 500, {.forwards=false}, true);
    setEffector(HIGH_ANGLE);
    startEffectorIntake();
    chassis.moveToPoint(47, 45, 1400, {.forwards=false, .maxSpeed=60, .minSpeed=30}, false);
    // Carry speed into the pickup and reach the pin before lowering.
    leftMotors.move(-30);
    rightMotors.move(-30);
    pros::delay(400);
    leftMotors.move(-20);
    rightMotors.move(-20);
    setEffector(IDLE_ANGLE);
    pros::delay(400);

    // Lift the effector clear before starting the goal turn.
    leftMotors.move(0);
    rightMotors.move(0);
    setEffector(RIGHT_ANGLE);
    pros::delay(500);
    chassis.turnToPoint(47, 23.5, 400, {.forwards=false}, true);
    chassis.moveToPoint(47, 23.5, 1000, {.forwards=false, .maxSpeed=80}, true);
    setLift(15);
    pros::delay(500);
    scoreCup(3, 20);
    pros::delay(1000);



}
