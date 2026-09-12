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

pros::Task* effectorMacroTask;
float effectorTargetDeg = 0.0;
float EFFECTOR_GEAR_RATIO = 4.0;
bool hardResettingEffector = false;

enum EFFECTOR_STAGES {IDLE, RIGHT_ANGLE, HIGH_ANGLE};
constexpr float EFFECTOR_ANGLES[3] = {0.0, 97.0, 110.0};

void initEffectorMacro() {
    effectorMacroTask = new pros::Task ([](){
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
        Timer t1(1000);
        while (!t1.timeIsUp() && effectorRotateMotor.get_actual_velocity() > 0.01) {pros::delay(50);}
        effectorRotateMotor.set_zero_position(0.0);
        hardResettingEffector = false;
        effectorRotateMotor.move(0);
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
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) raiseLift();
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) lowerLift();
    else stopLift();

    // Effector Rotation
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) setEffector(EFFECTOR_ANGLES[EFFECTOR_STAGES::HIGH_ANGLE]);
    else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) setEffector(EFFECTOR_ANGLES[EFFECTOR_STAGES::RIGHT_ANGLE]);
    else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) setEffector(EFFECTOR_ANGLES[EFFECTOR_STAGES::IDLE]);
    else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) hardResetEffector();
}