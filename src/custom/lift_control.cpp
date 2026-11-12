#include "custom/lift_control.hpp"
#include "custom/configs.hpp"
#include <atomic>
#include <queue>
#include "custom/util_funcs.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.h"
#include "pros/misc.h"
#include "string"


// BASIC MOTOR FUNCTIONS
void startFrontIntake(int velocity) {
    frontIntakeMotor.move(velocity);
}
void reverseFrontIntake(int velocity) {
    frontIntakeMotor.move(-velocity);
}
void stopFrontIntake() {
    frontIntakeMotor.move(0);
}

void startClawIntake(int velocity) {
    clawIntakeMotor.move(velocity);
}
void reverseClawIntake(int velocity) {
    clawIntakeMotor.move(-velocity);
}
void stopClawIntake() {
    clawIntakeMotor.move(0);
}

void openClaw(int velocity) {
    clawGrippersMotor.move(velocity);
}
void closeClaw(int velocity) {
    clawGrippersMotor.move(-velocity);
}
void stopClaw(int velocity) {
    clawGrippersMotor.move(0);
}

void raiseLift(int velocity) {
    liftMotors.move(velocity);
}
void lowerLift(int velocity) {
    liftMotors.move(-velocity);
}
void stopLift() {
    liftMotors.move(0);
}

pros::Task* liftMacroTask = nullptr;
float liftTargetHeight = 0.0;
constexpr float LIFT_GEAR_RATIO = 0.0;  // Deg / Inch height
void initLiftPID() {
    liftMacroTask = new pros::Task ([](){
        while (true) {
            if (enableLiftPID) {
                float output = liftPID.update(std::fmax(-60.0, liftTargetHeight*LIFT_GEAR_RATIO - liftMotors.get_position_all()[0]));
                liftMotors.move(output);
            }
            pros::delay(20);
        }
    });
}

void setLift(LIFT_STAGES newTargetEnum) {
    liftTargetHeight = LIFT_HEIGHTS[newTargetEnum];
}

void setLift(float newTargetInch) {
    liftTargetHeight = newTargetInch;
}

void resetLift() {
    liftTargetHeight = 0.0;
}

pros::Task L1Macro([](){});
bool effectorIntakeOccupied = false;
void updateLiftMotors() {

}

// Lift end effector and lift before calling this
void scoreCup(float dropHeight, float raiseHeight) {
    pros::Task([dropHeight, raiseHeight](){
        
    });
}

// Set effector to right angle before calling this
void scorePin() {
    pros::Task([](){
        
    });
}
