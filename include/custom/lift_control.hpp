#pragma once

#include "configs.hpp"

// ----------------- BASIC MOTOR CONTROL -----------------
void startFrontIntake(int velocity = 127);
void reverseFrontIntake(int velocity = 127);
void stopFrontIntake();

void startClawIntake(int velocity_rpm = 200);
void reverseClawIntake(int velocity_rpm = 200);
void stopClawIntake();

void openClaw(int velocity = 127);
void closeClaw(int velocity = 127);
void stopClaw();

void raiseLift(int velocity = 127);
void lowerLift(int velocity = 60);
void stopLift();

// ----------------- BASIC MOTOR MACROS -----------------
enum LIFT_STAGES {IDLE_HEIGHT, FIRST_STACK, SECOND_STACK, THIRD_STACK, FOUTH_STACK, FIFTH_STACK};
constexpr float LIFT_HEIGHTS[6] = {0.0, 10.0, 20.0, 30.0, 40.0, 50.0};    // Needs manual configuration

inline bool enableLiftPID = false;
inline float claw_open_position = 90.0;
inline float claw_close_position = 10.0;
inline bool clawHomed = false;

/*
    - Must be called within void initialize()
*/
void initLiftPID();

void setLift(LIFT_STAGES newTargetEnum);
void setLift(float newTargetInch);
void resetLift();
void hardResetClaw();

void updateLiftMotors();
