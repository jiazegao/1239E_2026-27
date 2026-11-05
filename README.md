# 1239E — 2026-27 Robot Code

Competition code for VEX V5 team **1239E**, 2026-27 season. Built with [PROS](https://pros.cs.purdue.edu/) and [LemLib](https://github.com/LemLib/LemLib).

## Stack

| Component | Version |
|-----------|---------|
| PROS kernel | 4.2.2 |
| LemLib | 0.5.6 |
| liblvgl | 9.2.0 |

## Building and uploading

Install the PROS CLI (or the PROS VS Code extension), then from the repo root:

```sh
pros make            # build
pros upload          # upload to the brain
pros mu              # build + upload
pros terminal        # view brain output
```

## Project layout

```
src/
  main.cpp                 initialize / autonomous / opcontrol entry points
  auton.cpp                autonomous routines
  custom/
    auton_selector.cpp     brain-screen auton selector (LVGL)
    lift_control.cpp       lift + end effector PID, macros, driver controls
    util_funcs.cpp         drive helpers, brain/controller displays, MCL startup, SD logging
    MclTracking.cpp        Monte Carlo localization (particle filter)
    RclTracking.cpp        distance-sensor pose resets against walls/obstacles
    FB_Logo.c              brain screen image
include/custom/
  configs.hpp              ports, drivetrain, PID constants, sensors, field obstacles
  ...                      headers for the files above, plus fast_trig.hpp and Tracking_Util.hpp
```

Everything under `include/lemlib`, `include/liblvgl`, `include/pros`, `include/fmt`, and `firmware/` comes from the PROS templates. Don't edit those files by hand.

## Hardware configuration

All ports live in `include/custom/configs.hpp`. A negative port means the motor is reversed.

| Device | Port(s) |
|--------|---------|
| Left drive (blue) | 10, 9 |
| Right drive (blue) | -19, -17 |
| Lift (blue) | -7, 21 |
| Front intake (blue) | 20 |
| End effector intake (green) | -5 |
| End effector rotate (green) | -4 |
| IMU | 16 |
| Optical sensor | 3 |
| Distance — front left / front right | 1 / 8 |
| Distance — left / back / right | 18 / 2 / 3 |

Drivetrain: 10.4" track width, 3.25" wheels, 450 RPM. The IMU is wrapped in `ScaledIMU`, which corrects its drift (it reads 360.91° per real 360°).

## Driver controls

Tank drive: left stick Y drives the left side, right stick Y drives the right side.

| Button | Action |
|--------|--------|
| R1 (hold) | Level the end effector, then raise the lift (the effector tilts near the top) |
| R2 (hold) | Lower the lift |
| L1 | Outtake the end effector briefly, then rotate it to the high angle |
| L2 (hold) | Reset the lift and end effector, run both intakes |
| B (hold) | Reverse both intakes |
| Down (hold) | Toggle angle + reverse the end effector intake |
| Y (hold) | Move the end effector to the pin angle |

When R1/R2 is released, the lift holds its height with PID.

## Autonomous

Pick a routine on the brain screen before the match. The selector has four buttons:

- **Color**: toggles Red / Blue alliance
- **Type**: cycles through the routines below
- **Skills**: when set to Y, runs `skills()` instead of the selected routine
- **Recal**: recalibrates the chassis/IMU

Routines (`src/auton.cpp`, registered in `include/custom/auton_selector.hpp`):

`Left 40`, `Right 40`, `Left 30`, `Right 30`, `Left 30_10`, `Right 30_10`, `Right 20_10`, `alliance1pin`, `skills`

To add a routine: write the function in `auton.cpp`, declare it in `auton.hpp`, and add it to `AutonCollection` (and update the array size).

## Localization

On top of LemLib odometry (IMU only, no tracking wheels), there are two systems:

- **RCL** (`RclTracking`): uses the five distance sensors to correct the robot's X/Y against the field walls, while ignoring readings blocked by known obstacles (field bases, auton-specific lines in `configs.hpp`).
- **MCL** (`MclTracking`): a 2048-particle Monte Carlo localization filter that weights particles by distance-sensor readings. You start it with `startMcl(x, y, theta, ...)`, which can do RCL resets first.

## Logging

`initLog()` (called in `initialize()`, don't remove it) writes MCL logs to the SD card as `/usd/mcl_log<N>.1239e`. The file number is incremented in `/usd/DATA.1239e`. If there's no SD card, the controller rumbles and logging turns off. Change `mclLogType` in `configs.hpp` to switch between `SDCARD`, `SCREEN`, or `DISABLED`.

## Tuning

The constants are in `configs.hpp` (drive lateral/angular PID, drive curves, effector and lift PID) and `lift_control.hpp` (effector angles, lift heights). `runPIDTuner()` in `util_funcs` is there to help with tuning.
