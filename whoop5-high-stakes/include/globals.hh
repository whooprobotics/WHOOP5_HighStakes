#ifndef GLOBALS_H  // these are header guards: https://www.geeksforgeeks.org/header-guard-in-c/#
#define GLOBALS_H

#include "rev/rev.hh"

// all lines that start with a # are directives: https://www.geeksforgeeks.org/cpp-preprocessors-and-directives/

// #define statements are used to define constants. 
// the first parameter is the name, and everything after it will be directly substituted in the code
// ex. #define PI 3.14159
//    double circumference = 2 * PI * radius;
// equivalent to
//    double circumference = 2 * 3.14159 * radius;
// definitions make code easier to read, however in C++, it is generally recommended to use const variables instead because they have type safety

// Ports
#define LEFT_MOTOR_GROUP {-11, -18, -13} // this is a vector, which is like a list in python or java
#define RIGHT_MOTOR_GROUP {}6, 5, 2
#define INTAKE {17, 7}

#define IMU_PORT 4    // these numbers correspond to the ports on the brain
#define FWD_PORT -1   // **when they are negative, it means that they are flipped. Ex. -1 for a motor will make motor go backwards when you input a positive value**
#define LAT_PORT -14

#define BEAM_BREAK_PORT 'H'   // 3-wire ports use characters instead of integers

#define BACK_WING_L_PORT 'B'
#define BACK_WING_R_PORT 'C'
#define FRONT_WINGS_PORT 'D'
#define ODOM_HYDRAULIC_PORT {16, 'G'}, true

// _mm, _in, _deg, ... are units. By using units, the compiler can check that your math is correct (ex. 1_deg + 1_mm is invalid)
// Parameters
#define WHEEL_DIAMETER 63.89_mm        // Diameter of forward and sideways wheel
#define FORWARD_WHEEL_OFFSET -1.125_in // How far to the right of the center of the robot the forward wheel is
#define LATERAL_WHEEL_OFFSET -1_in     // How far to the rear of the robot the lateral wheel is from the center


// tuning constants. These values are determined through a lot of testing
#define TURN_IKP1 0.18
#define TURN_IKP2 0.07

// AsyncRunners are used to run background threads that update controllers asyncrhonously.
// With background threads, the frontend user doesn't have to constantly update motor speeds, sensors, etc
// because it will be done automatically in the background
// https://pros.cs.purdue.edu/v5/tutorials/topical/multitasking.html
extern std::shared_ptr<rev::AsyncRunner> odom_runner;
extern std::shared_ptr<rev::AsyncRunner> reckless_runner;
extern std::shared_ptr<rev::AsyncRunner> turn_runner;

// TwoRotationInertialOdometry is the odometry system. It automatically tracks the robot's
// position and angle globally. 
// https://wiki.purduesigbots.com/software/odometry
extern std::shared_ptr<rev::TwoRotationInertialOdometry> odom;

// The chassis object represents the motors that are used to drive the robot around (not including intake, etc)
extern std::shared_ptr<rev::SkidSteerChassis> chassis;

// Reckless is the controller that drives the robot to points on the field.
// all points given to reckless are global positions. 
// Reckless does not care about your final angle. To go to a specific angle, you have to use the turn controller.
// the reckless_runner AsyncRunner controls the motors so the frontend code can do other stuff while reckless is driving
// because it is async though, you have to make sure not to send other motion controls to the chassis because it could cause conflicts
extern std::shared_ptr<rev::Reckless> reckless;

// This is the turn controller. It can only do point turns. It turns to a global angle. 
extern std::shared_ptr<rev::CampbellTurn> turn;

// motor ports
extern pros::MotorGroup left_motor_group; // the robots usually have a row of motors on the left and a row of motors on the right
extern pros::MotorGroup right_motor_group;
extern pros::MotorGroup intake;

// sensor inputs 
// https://pros.cs.purdue.edu/v5/api/cpp/imu.html?highlight=imu
extern pros::IMU imu;     // Intertial sensor. Measures acceleration and rotation using accelerometers and gyroscopes. For our odometry system, this is only used for angle

// https://pros.cs.purdue.edu/v5/api/cpp/rotation.html?highlight=rotation
extern pros::Rotation fwd; // rotation sensor parrellel to the wheels
extern pros::Rotation lat; // rotation sensor perpendicular to the wheels


// Beam Break
//https://pros.cs.purdue.edu/v5/api/cpp/adi.html
extern pros::ADIDigitalIn beam_break; // Consists of a light transmitter and a receiver. If the path between the two is blocked, then the sensor will output a 0, otherwise a 1. 

// hydraulics
//https://pros.cs.purdue.edu/v5/api/cpp/adi.html
extern pros::ADIDigitalOut backWingL;
extern pros::ADIDigitalOut backWingR;
extern pros::ADIDigitalOut frontWings;
extern pros::ADIDigitalOut odomHydraulic;

#endif // GLOBALS_H