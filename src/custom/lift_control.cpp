#include "custom/lift_control.hpp"
#include "custom/configs.hpp"
#include <atomic>
#include <queue>
#include "custom/util_funcs.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/misc.h"
#include "string"



pros::Task* effectorMacroTask;
float effectorTargetDeg = 0.0;
void initEffectorMacro() {
    effectorMacroTask = new pros::Task ([](){
        while (true) {
            float output = effectorPID.update(effectorTargetDeg-effectorRotateMotor.get_position());
            effectorRotateMotor.move(output);
            pros::delay(20);
        }
    });
}

bool hardResettingEffector = false;
void hardResetEffector() {
    pros::Task([](){
        hardResettingEffector = true;
        pros::delay(100);
        effectorTargetDeg = -1e99;
        pros::delay(3000);
        effectorRotateMotor.set_zero_position(0.0);
        hardResettingEffector = false;
    });
}

void setEffector(float newTargetDeg) {
    if (!hardResettingEffector) effectorTargetDeg = newTargetDeg;
}

void resetEffector() {
    if (!hardResettingEffector) effectorTargetDeg = 0.0;
}

void updateLiftMotors() {
    // Front Intake
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) startFrontIntake();
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) reverseFrontIntake();
    else stopFrontIntake();

    // Effector Intake
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) startEffectorIntake();
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) reverseEffectorIntake();
    else stopEffectorIntake();

    // Lift
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) raiseLift();
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) lowerLift();
    else stopLift();

    // Effector Rotation
    if (!hardResettingEffector) {
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) raiseEffector();
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) lowerEffector();
        else stopEffector();
    }
}