#ifndef GLOBALS_H  // these are header guards: https://www.geeksforgeeks.org/header-guard-in-c/#
#define GLOBALS_H

#include "rev/rev.hh"


// Smart Ports
#define LEFT_MOTOR_GROUP {-11, -18, -13, -12} // this is a vector, which is like a list in python or java
#define RIGHT_MOTOR_GROUP {6, 5, 2, 3}
#define INTAKE {17, -7}


#define IMU_PORT 4 
#define FWD_PORT -1   // **when they are negative, it means that they are flipped. Ex. -1 for a motor will make motor go backwards when you input a positive value**
#define LAT_PORT 14

// 3 wire ports
#define BEAM_BREAK_PORT          'H'
#define BACK_WING_L_PORT    {16, 'B'}, false
#define BACK_WING_R_PORT    {16, 'F'}, false
#define FRONT_WINGS_PORT    {16, 'H'}, false
#define ODOM_HYDRAULIC_PORT {16, 'G'}, true



// Odometry Parameters
#define WHEEL_DIAMETER 63.89_mm        // Diameter of forward and sideways wheel
#define FORWARD_WHEEL_OFFSET -1.125_in // How far to the right of the center of the robot the forward wheel is
#define LATERAL_WHEEL_OFFSET -1_in     // How far to the rear of the robot the lateral wheel is from the center



// tuning constants. These values are determined through a lot of testing
#define TURN_IKP1 0.18
#define TURN_IKP2 0.07


// background threads
extern std::shared_ptr<rev::AsyncRunner> odom_runner;
extern std::shared_ptr<rev::AsyncRunner> reckless_runner;
extern std::shared_ptr<rev::AsyncRunner> turn_runner;


// controllers
extern std::shared_ptr<rev::TwoRotationInertialOdometry> odom;
extern std::shared_ptr<rev::SkidSteerChassis> chassis;
extern std::shared_ptr<rev::Reckless> reckless;
extern std::shared_ptr<rev::CampbellTurn> turn;


// motor groups
extern pros::MotorGroup left_motor_group;
extern pros::MotorGroup right_motor_group;
extern pros::MotorGroup intake;


// sensor inputs 
extern pros::IMU imu;
extern pros::Rotation fwd;
extern pros::Rotation lat;


// Beam Break
extern pros::ADIDigitalIn beam_break;  


// hydraulics
extern pros::ADIDigitalOut backWingL;
extern pros::ADIDigitalOut backWingR;
extern pros::ADIDigitalOut frontWings;
extern pros::ADIDigitalOut odomHydraulic;

#endif // GLOBALS_H