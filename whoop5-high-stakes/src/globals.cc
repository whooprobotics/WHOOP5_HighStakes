#include "globals.hh"
#include "rev/rev.hh"

std::shared_ptr<rev::AsyncRunner> odom_runner;
std::shared_ptr<rev::AsyncRunner> reckless_runner;
std::shared_ptr<rev::AsyncRunner> turn_runner;

std::shared_ptr<rev::TwoRotationInertialOdometry> odom;

std::shared_ptr<rev::Reckless> reckless;
std::shared_ptr<rev::CampbellTurn> turn;

// motor ports
pros::MotorGroup left_motor_group(LEFT_MOTOR_GROUP);
pros::MotorGroup right_motor_group(RIGHT_MOTOR_GROUP);
std::shared_ptr<rev::SkidSteerChassis> chassis = std::make_shared<rev::SkidSteerChassis>(left_motor_group, right_motor_group);


pros::MotorGroup intake(INTAKE);


// sensor inputs
pros::IMU imu(IMU_PORT);
pros::Rotation fwd(FWD_PORT);
pros::Rotation lat(LAT_PORT);

// Beam Break
pros::ADIDigitalIn beam_break(BEAM_BREAK_PORT);

// hydraulics
pros::ADIDigitalOut backWingL(BACK_WING_L_PORT);
pros::ADIDigitalOut backWingR(BACK_WING_R_PORT);
pros::ADIDigitalOut frontWings(FRONT_WINGS_PORT);
pros::ADIDigitalOut odomHydraulic(ODOM_HYDRAULIC_PORT);
