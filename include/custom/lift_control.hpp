#pragma once

#include "configs.hpp"

// ----------------- BASIC MOTOR CONTROL -----------------
void startFrontIntake(int velocity = 127);
void reverseFrontIntake(int velocity = 127);
void stopFrontIntake();

void startEffectorIntake(int velocity = 127);
void reverseEffectorIntake(int velocity = 127);
void stopEffectorIntake();

void raiseLift(int velocity = 127);
void lowerLift(int velocity = 127);
void stopLift();

void raiseEffector(int velocity = 127);
void lowerEffector(int velocity = 127);
void stopEffector();

// ----------------- BASIC MOTOR MACROS -----------------
void initEffectorMacro();
/*
    - Must be called within void initialize()
    - Starts the pros::Task that manages effector setpose macro
*/
void setEffector(float newTargetDeg);
void resetEffector();
void hardResetEffector();

void updateLiftMotors();
