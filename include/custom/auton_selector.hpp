#pragma once

#include "auton.hpp"
#include "main.h" // IWYU pragma: keep

// Autonomous selection variables
inline bool runningSkills = false;
inline bool autonMoveToPose = false;

using voidFunc = void(*)();
struct AutonEntry {
    std::string_view Name;
    voidFunc AutonFunc;
};

// Auton Collection
inline int autonCount = 1;
constexpr std::array<AutonEntry, 1> AutonCollection = {{
    {"EXAMPLE", nullptr},
}};

// GUI objects
inline lv_obj_t* label_color;
inline lv_obj_t* label_type;
inline lv_obj_t* label_skills;
inline lv_obj_t* btn_color;
inline lv_obj_t* btn_type;
inline lv_obj_t* btn_skills;
inline lv_obj_t* btn_recalibrate;
inline lv_obj_t* label_recalibrate;

// Button event callback functions
void toggle_color(lv_event_t* e);

void toggle_type(lv_event_t* e);

void toggle_skills (lv_event_t* e);

void recalibrate(lv_event_t* e);

// Initialize autonomous selector GUI
void init_auton_selector();

void runAuton();