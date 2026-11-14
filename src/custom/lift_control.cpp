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
    frontIntakeMotor.brake();
}

void startClawIntake(int velocity_rpm) {
    clawIntakeMotor.move_velocity(velocity_rpm);
}
void reverseClawIntake(int velocity_rpm) {
    clawIntakeMotor.move_velocity(-velocity_rpm);
}
void stopClawIntake() {
    clawIntakeMotor.brake();
}

void openClaw(int velocity) {
    if (!clawHomed) return;
    clawGrippersMotor.move_absolute(claw_open_position, velocity);
}
void closeClaw(int velocity) {
    if (!clawHomed) return;
    clawGrippersMotor.move_absolute(claw_close_position, velocity);
}
void stopClaw() {
    if (!clawHomed) return;
    clawGrippersMotor.brake();
}

void raiseLift(int velocity) {
    liftMotors.move(velocity);
}
void lowerLift(int velocity) {
    liftMotors.move(-velocity);
}
void stopLift() {
    liftMotors.brake();
}

pros::Task* liftMacroTask = nullptr;
float liftTargetHeight = 0.0;
std::atomic<bool> liftResetting{false};  // true while lift drives to bottom
constexpr float LIFT_GEAR_RATIO = 0.0;  // Deg / Inch height
void initLiftPID() {
    liftMacroTask = new pros::Task ([](){
        bool wasEnabled = false;
        while (true) {
            bool active = enableLiftPID && !liftResetting;
            if (active) {
                float error = std::fmax(-60.0, liftTargetHeight*LIFT_GEAR_RATIO - liftMotors.get_position_all()[0]);
                if (!wasEnabled) {            // just took over: clear integral, no derivative kick
                    liftPID.reset();
                    liftPID.prevError = error;
                }
                float output = liftPID.update(error);
                if (std::fabs(output) > 1.0f) liftMotors.move(output);
                else liftMotors.brake();
            }
            wasEnabled = active;
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

pros::Task liftResetTask([](){});
void resetLift() {
    liftTargetHeight = 0.0;
    if (liftResetting) return;  // already going down, don't start again
    liftResetting = true;
    liftResetTask = pros::Task([](){
        liftMotors.move(-80);
        pros::delay(200);  // let it start moving
        uint32_t start = pros::millis();
        // drive down until lift stalls on bottom, or 1.5 s pass
        while (pros::millis() - start < 1500 &&
               std::fabs(liftMotors.get_actual_velocity()) > 5) {
            pros::delay(20);
        }
        liftMotors.brake();
        liftMotors.tare_position_all();  // bottom = 0
        liftResetting = false;
    });
}

pros::Task clawResetTask([](){});
void hardResetClaw() {
    clawHomed = false;
    clawResetTask = pros::Task([](){
        clawGrippersMotor.move(-127);
        pros::delay(1000);
        clawGrippersMotor.brake();
        clawGrippersMotor.set_zero_position(0.0);
        clawHomed = true;
    });
}

float passive_height = 0.0;
bool liftOnPID = false;

void updateLiftMotors() {

    // DOWN lift at set "passive" toggle height
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        liftOnPID = true;
        setLift(passive_height);
    }
    /*RIGHT spin claw roller intake, rotate grippers to grab position,
    keep claw rollers intaking at 50 rpm until L1, or B are pressed
    */
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
    closeClaw();
        startFrontIntake(127);
        startClawIntake(127);
    }

    //L2 reset lift spin intake, spin claw roller intake,
    // rotate grippers to grab position, keep claw rollers intaking at 50 rpm until L1,
    // or B are pressed
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
        liftOnPID = true;
        resetLift();
        closeClaw();
        startFrontIntake(127);
        startClawIntake(127);
    }
    /*L1 open grippers to open position, outtake claw roller at full speed*/
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
        reverseClawIntake();
        openClaw();
    }
    // B outtake claw roller, reverse/outtake ground intake
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
        reverseClawIntake();
        openClaw();
        reverseFrontIntake();
    }
    else {
        stopFrontIntake();
    }

    // L2 let go -> claw rollers back to 50 rpm intake
    static bool prevL2 = false;
    bool l2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
    bool b  = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);
    if (prevL2 && !l2 && !b) startClawIntake(50);
    prevL2 = l2;

    if (liftResetting) return;  // reset task owns the lift right now

    bool r1 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
    bool r2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);

    if (r1 || r2) liftOnPID = false;
    enableLiftPID = liftOnPID;
    if (r2) lowerLift();
    else if (r1) raiseLift();
    else if (!liftOnPID) stopLift();
}
