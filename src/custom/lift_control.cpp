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

void startEffectorIntake(int velocity) {
    effectorIntakeMotor.move(velocity);
}
void reverseEffectorIntake(int velocity) {
    effectorIntakeMotor.move(-velocity);
}
void stopEffectorIntake() {
    effectorIntakeMotor.move(0);
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

void raiseEffector(int velocity) {
    effectorRotateMotor.move(velocity);
}
void lowerEffector(int velocity) {
    effectorRotateMotor.move(-velocity);
}
void stopEffector() {
    effectorRotateMotor.move(0);
}

pros::Task* effectorPIDTask = nullptr;
float effectorTargetDeg = 0.0;
constexpr float EFFECTOR_GEAR_RATIO = 4.0;  // Deg motor / Deg end effector
bool hardResettingEffector = false;
void initEffectorPID() {
    effectorPIDTask = new pros::Task ([](){
        while (true) {
            if (!hardResettingEffector) {
                float output = effectorPID.update(effectorTargetDeg*EFFECTOR_GEAR_RATIO - effectorRotateMotor.get_position());
                effectorRotateMotor.move(output);
            }
            pros::delay(20);
        }
    });
}

void hardResetEffector() {
    pros::Task([](){
        hardResettingEffector = true;
        pros::delay(200);
        effectorRotateMotor.move(-127);
        pros::delay(1000);
        effectorRotateMotor.set_zero_position(0.0);
        hardResettingEffector = false;
        effectorTargetDeg = 0.0;
        effectorRotateMotor.move(0);
    });
}

void setEffector(EFFECTOR_STAGES newTargetEnum) {
    if (!hardResettingEffector) effectorTargetDeg = EFFECTOR_ANGLES[newTargetEnum];
}

void setEffector(float newTargetDeg) {
    if (!hardResettingEffector) effectorTargetDeg = newTargetDeg;
}

void resetEffector() {
    if (!hardResettingEffector) effectorTargetDeg = 0.0;
}

pros::Task* liftMacroTask = nullptr;
float liftTargetHeight = 0.0;
constexpr float LIFT_GEAR_RATIO = 31.706;  // Deg / Inch height
bool hardResettingLift = false;
void initLiftPID() {
    liftMacroTask = new pros::Task ([](){
        while (true) {
            if (!hardResettingLift) {
                float output = liftPID.update(liftTargetHeight*LIFT_GEAR_RATIO - liftMotors.get_position());
                liftMotors.move(output);
            }
            pros::delay(20);
        }
    });
}

void setLift(LIFT_STAGES newTargetEnum) {
    if (!hardResettingLift) liftTargetHeight = LIFT_HEIGHTS[newTargetEnum];
}

void setLift(float newTargetInch) {
    if (!hardResettingLift) liftTargetHeight = newTargetInch;
}

void resetLift() {
    if (!hardResettingLift) liftTargetHeight = 0.0;
}

pros::Task* L2Macro = nullptr;
pros::Task* L1Macro = nullptr;
bool L2MacroInitializing = false;
void updateLiftMotors() {

    // L2 Macro - Reset lift & end effector, then intake and spin end effector
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
        if (L2Macro != nullptr) L2Macro->remove();
        L2Macro = new pros::Task([](){
            L2MacroInitializing = true;
            stopEffectorIntake();
            stopFrontIntake();
            resetLift();
            resetEffector();
            pros::delay(500);
            startFrontIntake();
            startEffectorIntake();
            L2MacroInitializing = false;
        });
    }

    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2) && !L2MacroInitializing) {
        startFrontIntake();
        startEffectorIntake();
    }

    // L1 Macro - Outtake end effector for 500ms, then lift up end effector
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
        if (L1Macro != nullptr) L1Macro->remove();
        L1Macro = new pros::Task([](){
            reverseEffectorIntake();
            pros::delay(500);
            setEffector(HIGH_ANGLE);
        });
    }

    // R1 Macro - Prime end effector (horizontal), then lift on hold
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
        setEffector(RIGHT_ANGLE);
        raiseLift();
    }

    // R2 Macro - Lower lift on hold
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
        lowerLift();
    }

    // B Macro - Outtake both front intake and end effector
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
        reverseFrontIntake();
        reverseEffectorIntake();
    }

    // Down Macro - Toggle using end effector
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        setEffector(TOGGLE_ANGLE);
        reverseEffectorIntake(60);
    }
}

void effectorToggle() {
    pros::Task([](){
        reverseEffectorIntake();
        pros::delay(800);
        stopEffectorIntake();
    });
}

void scoreCup() {
    pros::Task([](){
        setEffector(IDLE_ANGLE);
        pros::delay(400);
        reverseEffectorIntake();
        pros::delay(100);
        setEffector(HIGH_ANGLE);
        pros::delay(400);
        stopEffectorIntake();
    });
}

void scorePin() {
    pros::Task([](){
        setEffector(RIGHT_ANGLE);
        pros::delay(400);
        reverseEffectorIntake();
        pros::delay(400);
        stopEffectorIntake();
    });
}