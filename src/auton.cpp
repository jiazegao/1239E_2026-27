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

void testPid(){
    chassis.setPose(0, 0, 0);
    moveForward(24, 1500, 127, 1, false);
    chassis.turnToHeading(90, 1500, {}, false);
    moveForward(-24, 1500, 127, 1, false);
    chassis.turnToHeading(270, 1500, {}, false);
}
void toggle() {
    moveForward(15, 400, 127, 100, true);
    moveForward(-5, 300, 127, 60, true);
}

void left3pin(){
     chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(60, 0, 90);
    closeClaw();
    pros::delay(200);
   // MclMain.setObstacles(&leftAutonObstacles, nullptr);
    // startMcl(60, 0, 90, true, false, false, false);
    // startFrontIntake();
    
    enableLiftPID = true;     // lift PID holds the target height
    resetLift();             // drive to bottom + zero encoder; PID waits until it's done
    setLift(LIFT_HEIGHTS[THIRD_STACK] + 200.0f/7); // THIRD_STACK + 200 motor deg
    startClawIntake(80);  // start intaking the pin
    pros::delay(1500);
//Move to alliance goal and pin drop
    chassis.moveToPoint(44, 0, 2000, { .forwards = false, .maxSpeed = 100,}, true);
    setLift(FIRST_STACK); 
    chassis.turnToPoint(44, 24, 600, {.forwards = false}, true);
    chassis.moveToPoint(46, 24, 400, {.forwards = false, .maxSpeed = 80}, true);

    chassis.moveToPoint(44, 24, 2000, {.forwards = false, .maxSpeed = 50}, true);
    //Drop pin and back up
   pros::delay(800);
    setLift(LIFT_HEIGHTS[IDLE_HEIGHT]);  // drop pin
   pros::delay(200);
       openClaw(-150, 127);  // open wider for the drop
   reverseClawIntake(50);
    pros::delay(600);
    //back up
    chassis.moveToPoint(44, 10, 1500, { .maxSpeed = 100}, true);
   //get cup and score
    chassis.turnToPoint(23, 24, 600, {.forwards = false}, true);
    chassis.moveToPoint(23, 24, 400, {.forwards = false, .minSpeed = 70 }, true);
    chassis.moveToPoint(23, 24, 2000, { .forwards = false, .maxSpeed = 45,});
    startClawIntake(127);
    closeClaw(10, 127);
    pros::delay(300);
    //score
    setLift(SECOND_STACK);
    pros::delay(300);
    chassis.turnToPoint(44, 34, 600, {.forwards = false}, true);
    pros::delay(200);
    chassis.moveToPoint(44, 34, 400, {.forwards = false, .minSpeed = 70 }, true);
    chassis.moveToPoint(44, 34, 2000, { .forwards = false, .maxSpeed = 45,});
    pros::delay(200);
    setLift(LIFT_HEIGHTS[IDLE_HEIGHT]);
    openClaw(-150, 127);
}