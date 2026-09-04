#pragma once

#include "lemlib/chassis/chassis.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp" // IWYU pragma: keep
#include "pros/distance.hpp" // IWYU pragma: keep
#include "pros/imu.hpp" // IWYU pragma: keep
#include "pros/misc.hpp" // IWYU pragma: keep
#include "pros/motor_group.hpp" // IWYU pragma: keep
#include "pros/motors.hpp"   // IWYU pragma: keep
#include "pros/optical.hpp"     // IWYU pragma: keep
#include "pros/rotation.hpp" // IWYU pragma: keep
#include <cmath>
#include <numbers>

#include "custom/RclTracking.hpp"
#include "custom/MclTracking.hpp"

#include <cmath>

class ScaledIMU : public pros::Imu {
private:
    double scale_factor;

public:
    // Constructor: Passes the port to the base pros::Imu class and calculates the scale factor
    ScaledIMU(uint8_t port, double expected_rotation = 360.0, double actual_reading = 355.0) 
        : pros::Imu(port) {
        scale_factor = expected_rotation / actual_reading; 
    }

    // "Override" the rotation method to return the scaled continuous rotation
    double get_rotation() const {
        double raw_rotation = pros::Imu::get_rotation();
        
        // Check for PROS error return (usually INFINITY for doubles)
        if (std::isinf(raw_rotation)) {
            return raw_rotation; 
        }
        
        return raw_rotation * scale_factor;
    }

    // "Override" the heading method to return a bounded 0-360 degree value
    double get_heading() const {
        double scaled_rotation = this->get_rotation();
        
        if (std::isinf(scaled_rotation)) {
            return scaled_rotation;
        }

        // Mathematically bound the scaled rotation to 0-360
        double heading = std::fmod(scaled_rotation, 360.0);
        if (heading < 0) {
            heading += 360.0;
        }
        
        return heading;
    }
};

const int tempPort = 21;

// Alliance Color
enum class alliance_color { RED, BLUE, NONE };
inline alliance_color allianceColor = alliance_color::BLUE;

// Controller
inline pros::Controller controller(pros::E_CONTROLLER_MASTER);
inline pros::Controller partner_controller(pros::E_CONTROLLER_PARTNER);

// Motors
inline pros::MotorGroup leftMotors({tempPort, tempPort}, pros::MotorGearset::blue);
inline pros::MotorGroup rightMotors({tempPort, tempPort}, pros::MotorGearset::blue);
inline pros::MotorGroup liftMotors({tempPort, tempPort}, pros::MotorGearset::red);
inline pros::Motor frontIntakeMotor(tempPort, pros::MotorGearset::blue);
inline pros::Motor effectorIntakeMotor(tempPort, pros::MotorGearset::red);
inline pros::Motor effectorRotateMotor(tempPort, pros::MotorGearset::red);
inline lemlib::PID effectorPID(0.0, 0.0, 0.0);

inline lemlib::Drivetrain drivetrain(&leftMotors,
                              &rightMotors,
                              10.400,
                              3.25,
                              450,
                              2
);

// IMU
inline ScaledIMU imu(15, 360.0, 354.25); // Adjust actual_reading based on your IMU's behavior

// Optical
inline pros::Optical frontOptic(3);

// Odometry
inline lemlib::OdomSensors sensors( nullptr,
                                    nullptr,
                                    nullptr,
                                    nullptr,
                                    &imu
);

// Lateral PID controller
inline lemlib::ControllerSettings lateral_controller(
                                              7.0, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              30.0, // derivative gain (kD)
                                              0, // anti windup
                                              0.5, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              1.5, // large error range, in inches
                                              200, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// Angular PID controller
inline lemlib::ControllerSettings angular_controller(4.0, // proportional gain (kP)
                                            0, // integral gain (kI)
                                              37.7, // derivative gain (kD)
                                              0, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              2, // large error range, in degrees
                                              200, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// Throttle curve   
inline lemlib::ExpoDriveCurve throttle_curve(
    15,    // deadband
    20,    // minOutput
    1.05   // curve
);

// Steer curve
inline lemlib::ExpoDriveCurve steer_curve(
    10,    // deadband
    30,    // minOutput
    1.3    // curve
);


// Chassis
inline lemlib::Chassis chassis( drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);

// Distance
inline pros::Distance midDist(tempPort);
inline pros::Distance topDist(tempPort);

enum DISTSENSORS {FRONT, LEFT, BACK, RIGHT, FRONT_LEFT, BACK_LEFT, BACK_RIGHT, FRONT_RIGHT};
inline pros::Distance front_dist(tempPort);
inline pros::Distance left_dist(tempPort);
inline pros::Distance back_dist(tempPort);
inline pros::Distance right_dist(tempPort);


inline std::array<pros::Distance*, 4> DISTANCE_COLLECTION = {&front_dist, &left_dist, &back_dist, &right_dist};

// Rcl setup
inline RclSensor front_rcl(&front_dist, -2.277110, 6.184952, 0, 15.0);
inline RclSensor left_rcl(&left_dist, -2.674094, -0.733924, 270.0, 15.0);
inline RclSensor back_rcl(&back_dist, 1.75, -5.374061, 180.0, 15.0);
inline RclSensor right_rcl(&right_dist, 2.674094, -0.733924, 90.0, 15.0);
inline RclTracking RclMain(&chassis, 1, false, 0.5, 4.0, 200.0, 6.0, 50);
inline MclTracking MclMain(&chassis, &drivetrain, DISTANCE_COLLECTION, {nullptr, 0.0, 0.0}, {nullptr, 0.0, 0.0}, 0, 0, 0, true);

enum MCL_Log_Format {DISABLED, SDCARD, SCREEN};
inline MCL_Log_Format mclLogType = DISABLED;
inline std::ofstream* mclLog = nullptr;
inline Timer mclLogTimer(100000000.0f);

// Mcl obstacles
/* ADD LATER BASED ON NEED*/

// Disable Line
/* ADD LATER BASED ON NEED*/