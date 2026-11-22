# 1239E — 2026-27 Robot Code (`six-bar` branch)

Competition code for VEX V5 team **1239E**, 2026-27 season. Built with [PROS](https://pros.cs.purdue.edu/) and [LemLib](https://github.com/LemLib/LemLib).

This branch is for the robot with the six-bar lift and claw. `main` has the season's earlier robot, which uses different ports, mechanisms, and auton routines, so its README and configs don't apply here.

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
    lift_control.cpp       lift PID and reset, claw, intakes, driver controls
    util_funcs.cpp         drive helpers, displays, MCL startup, SD logging, PID tuner
    MclTracking.cpp        Monte Carlo localization (particle filter)
    RclTracking.cpp        distance-sensor pose resets against the walls
    FB_Logo.c              brain screen image
include/custom/
  configs.hpp              ports, drivetrain, PID constants, sensors, obstacles
  lift_control.hpp         lift heights, lift hard stop, claw positions
  auton_selector.hpp       list of selectable routines
  ...                      headers for the files above, plus fast_trig.hpp and Tracking_Util.hpp
```

Everything under `include/lemlib`, `include/liblvgl`, `include/pros`, `include/fmt`, and `firmware/` comes from the PROS templates. Don't edit those files by hand.

## Hardware configuration

All ports live in `include/custom/configs.hpp`. A negative port means the device is reversed.

| Device | Port(s) |
|--------|---------|
| Left drive | -15, -14, -13 |
| Right drive | 8, 7, 10 |
| Lift (blue) | -19, 20 |
| Front intake (blue) | 17 |
| Claw gripper (green) | 18 |
| Claw rollers (green) | -16 |
| IMU | 6 |
| Vertical tracking wheel (rotation sensor) | -9 |
| Distance — front / back | 2 / 21 |
| Distance — left / right | 3 / 5 |
| Distance — back-left / back-right | 4 / 11 |

- **Drivetrain:** three motors per side. The first two are 11W blue motors; the third (ports 13 and 10) is a 5.5W motor, and `initialize()` sets its gearing to green. 2.75" wheels, 450 RPM, 11.41" track width.
- **Odometry:** a 3.25" vertical tracking wheel on the rotation sensor (no horizontal wheel), plus the IMU.
- **IMU:** wrapped in `ScaledIMU`, which can correct drift. It's set to 360/360 (no correction) right now. To correct it, turn the robot exactly 360° and enter what the IMU reads as `actual_reading`.
- **Lift:** 12:84 gearing (7 motor degrees per lift-arm degree). Motor 19 is always plugged in and is used for position. Motor 20 is only plugged in for skills.

## Driver controls

Tank drive: left stick Y drives the left side, right stick Y drives the right side. The drive coasts in driver control.

**Lift**

| Button | Action |
|--------|--------|
| R1 (hold) | Raise the lift. Stops at the hard stop (`LIFT_MAX_MOTOR_DEG`). |
| R2 (hold) | Lower the lift: power 76 when both lift motors are plugged in, 127 otherwise |
| Down | Hold the lift at the bottom with PID |

When R1/R2 is released, the lift holds its height with the motor brake (HOLD), not PID. R1 or R2 also cancels a lift reset that's in progress.

**Intakes and claw.** These are checked in this order, and only the first held button acts:

| Button | Action |
|--------|--------|
| L2 (hold) | On press, reset the lift to the bottom and hold it there. While held, close the claw and run the front intake and claw rollers. On release, the rollers keep intaking at 50 rpm. |
| L1 (hold) | Open the claw, reverse the claw rollers |
| B (hold) | Open the claw, stop the claw rollers, reverse the front intake |
| Right (hold) | Open the claw, run the front intake, run the rollers at 50 rpm |

With none of these held, the front intake stops and the claw rollers keep their last command.

**Other**

| Button | Action |
|--------|--------|
| Y (hold) | Open the claw wide (200°) |
| Up + X | Emergency reset: stop the intakes, re-home the claw, reset the lift to the bottom |

**Claw homing:** `hardResetClaw()` drives the gripper closed for 0.5 s and calls that position 0°. Open is 140° and closed is 10° (`lift_control.hpp`). Driver control homes the claw when it starts. `autonomous()` doesn't; it assumes the gripper starts at 13°.

## Autonomous

Pick a routine on the brain screen before the match. The selector has four buttons:

- **Color**: toggles Red / Blue alliance (defaults to Blue)
- **Type**: cycles through the routines in `AutonCollection`
- **Skills**: when set to Y, runs `safe_skills()` instead of the selected routine
- **Recal**: recalibrates the chassis/IMU

In autonomous the drive brakes with HOLD and the lift PID is on.

Routines (`src/auton.cpp`). Coordinates are in inches from the field center; headings are LemLib degrees (0° = +Y, clockwise positive).

| Function | In selector | What it does |
|----------|-------------|--------------|
| `left3pin` | Yes ("LEFT 3-PIN") | Starts at (57, 0, 90°). Drops the preloaded pin at the alliance goal, then picks up a cup at (20, 25) and scores it at (44, 34). Runs MCL with `leftAutonObstacles`. |
| `safe_skills` | Through Skills = Y | Same opening as `left3pin`, then drives toward (0, 0) |
| `left3pinv2` | No | `left3pin` with adjusted timing, plus a second cup from (47, 47) |
| `skills` | No | Starts at (-59, -35, 225°). Scores a pin, then runs six pick-up-and-stack cycles. Doesn't use MCL. |
| `testPid` | No | Drives and turns a short test pattern |

To add a routine: write the function in `auton.cpp`, declare it in `auton.hpp`, and add it to `AutonCollection` in `auton_selector.hpp` (and update the array size).

## Localization

LemLib odometry (vertical tracking wheel + IMU) is the base. Two systems use the six distance sensors to correct it:

- **MCL** (`MclTracking`): a 2048-particle filter that runs about every 30 ms. It compares each sensor's reading against a map of the field (walls, match loaders, and goal bases, defined in `MclTracking.hpp`) and moves LemLib's X/Y 5% toward its estimate each cycle. If a sensor reads shorter than the map predicts, a Pin, Cup, or robot may be in the way, so that penalty is capped (`SHORT_READING_FLOOR` in `MclTracking.hpp`). Heading always comes from the IMU. Start it with `startMcl(x, y, theta, resetFront, resetLeft, resetBack, resetRight)`; the flags do one-shot RCL wall resets first. Change how strongly it corrects with `MclMain.setDistSyncProp()` (0 turns correction off).
- **RCL** (`RclTracking`): sets X or Y from one sensor's distance to a wall, ignoring readings blocked by the bases and match loaders listed in `configs.hpp`. It's used through `startMcl()`'s reset flags.

`MclMain.setObstacles(&leftAutonObstacles)` makes MCL ignore sensor readings whose rays cross the lines in `leftAutonObstacles` or `rightAutonObstacles` (`configs.hpp`).

Sensor positions and field elements are each listed twice: in `configs.hpp` for RCL (`RclSensor` objects and obstacle circles) and in `MclTracking.hpp` for MCL (`sensor_mounts` and the field map). If you move a sensor or change the field map, update both.

To run on odometry alone, set `distanceSensorsEnabled = false` in `configs.hpp`.

## Logging

`initLog()` (called in `initialize()`, don't remove it) sets up MCL logging. Choose the mode with `mclLogType` in `configs.hpp`:

- `SDCARD`: writes MCL logs to the SD card as `/usd/mcl_log<N>.1239e`. The file number is incremented in `/usd/DATA.1239e`. If there's no SD card, the controller rumbles and logging turns off.
- `SCREEN` (current setting): prints MCL computation time with `pros::lcd`. This only appears if `pros::lcd::initialize()` is called, and that call is commented out in `initialize()`.
- `DISABLED`: no logging.

## Tuning

The constants are in:

- `configs.hpp`: drive lateral and angular PID, throttle and steer curves, lift PID (`liftPID`)
- `lift_control.hpp`: lift heights (`LIFT_HEIGHTS`, in lift-arm degrees, written as motor degrees / 7), the lift hard stop (`LIFT_MAX_MOTOR_DEG`), and the claw open/closed positions

`runPIDTuner()` in `util_funcs.cpp` tunes the drive PID from the controller. Nothing calls it, so call it from `opcontrol()` while tuning. It shows values with `pros::lcd`, so call `pros::lcd::initialize()` first. It loads values from `/usd/PIDValues.1239e` when it starts and saves them after each test move.

| Button | Action |
|--------|--------|
| A | Switch between lateral and angular |
| Up / Down (hold) | kP ± 0.1 |
| Right / Left | kI ± 0.05 |
| X / B (hold) | kD ± 0.1 |
| L2 / L1 | Increase / decrease the test distance (2") or angle (5°) |
| R2 | Drive the test distance |
| R1 | Turn the test angle |
| Y | Return to (0, 0), facing 0° |
