// #include directly copies the content of the specified file into the place where the #include directive is used.
// when including standard C++ libraries, you use angle brackets. When including your own files, you use quotes.
//https://en.cppreference.com/w/cpp/preprocessor/include
#include "main.h"
#include "pros/motors.hpp"
#include "rev/rev.hh"
#include "globals.hh"

#include <string>

// this makes it so you can just put the name of the item rather then putting rev:: before everything
using namespace rev;

void print_position();

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();

	chassis = std::make_shared<rev::SkidSteerChassis>(left_motor_group, right_motor_group);

	odom = std::make_shared<rev::TwoRotationInertialOdometry>(
    fwd,      // The forward sensor
    lat,      // The rightward sensor 
    imu,      // Inertial sensor
    WHEEL_DIAMETER,  // Diameter of forward wheel
    WHEEL_DIAMETER,  // Diameter of sideways wheel
    FORWARD_WHEEL_OFFSET,  // How far to the right of the center of the robot the forward wheel is
    LATERAL_WHEEL_OFFSET    // How far to the rear of the robot the lateral wheel is from the center
  );


  // creates a turn controller object. This turn controller can only do point turns
  turn = std::make_shared<CampbellTurn>(chassis, odom, TURN_IKP1, TURN_IKP2);


  // creates a reckless controller object. This is used to drive the robot to points on the field
  reckless = std::make_shared<Reckless>(chassis, odom);

  pros::delay(2000);
  }

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	odomHydraulic.set_value(0);

    odom_runner = std::make_shared<rev::AsyncRunner>(odom);
	reckless_runner = std::make_shared<rev::AsyncRunner>(reckless);
	turn_runner = std::make_shared<rev::AsyncRunner> (turn);

	odom->reset_position();


	print_position();


	reckless->go(
		RecklessPath().with_segment(
		RecklessPathSegment(
			std::make_shared<ConstantMotion>(0.5),             // tells the robot to move at 50% power
			std::make_shared<PilonsCorrection>(4, 0.3_in), // if the robot is 0.3in or more off the path, then it will start correcting that path
			std::make_shared<SimpleStop>(0.03_s, 0.15_s, 0.3),   // robot will soft stop if it is 0.15 seconds from the finish, and hard stop when it is 0.03 seconds from the finish. Soft stop means that the speed is set to 30% power. Hard stop means that the brakes are applied
			{ 20_in, 0_in, 0_deg },               // the target global position. Position 0, 0 is where the robot starts. the 0_deg is meaningless but it has to be included for syntax reasons
			0_in)                                          // tells the robot to stop 0_in from the target
	));

	while (!reckless->is_completed()) {
		print_position();
		pros::delay(20);
	}

	// turn the robot 90 degrees with a max power of 70%
	turn->turn_to_target_absolute(0.7, 90_deg);
	
	while (!turn->is_completed()) {
		print_position();
		pros::delay(20);
	}
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	pros::Motor_Group left_group LEFT_MOTOR_GROUP;
	pros::Motor_Group right_group RIGHT_MOTOR_GROUP;
	pros::Motor_Group intake_group INTAKE;


	pros::Controller master(pros::E_CONTROLLER_MASTER); // used to get inputs from the users's controller


    while (true) {
		int left = master.get_analog(ANALOG_LEFT_Y);
		int right = master.get_analog(ANALOG_RIGHT_Y);

		left_group = left;
		right_group = right;

		pros::delay(20);
	}












/*
	pros::lcd::set_text(0, "Hello PROS User!"); // shows text on the LCD. Useful for testing purposes. 
	pros::Controller master(pros::E_CONTROLLER_MASTER); // used to get inputs from the users's controller

	while (true) {
		// read from the controller the the left and right joystick values. These values are from -127 to 127, which the same range used by the motors. 
		int rightX = master.get_analog(ANALOG_RIGHT_X);
		int leftY = master.get_analog(ANALOG_LEFT_Y);

		// set the motors powers to be the same as the joystick values
		left_motor_group = leftY + rightX;
		right_motor_group = leftY - rightX;
		
		// delay 20 milliseconds. **This is important because it gives the background threads a change to run**
		pros::delay(20);
	}*/
}


void print_position() {
	std::string position = "Position: ";
	position += std::to_string(odom->get_state().pos.x.convert(inch));
	position += ", ";
	position += std::to_string(odom->get_state().pos.y.convert(inch));
	position += ", ";
	position += std::to_string(odom->get_state().pos.theta.convert(degree));

	pros::lcd::set_text(2, position);
}