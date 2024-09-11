// #include directly copies the content of the specified file into the place where the #include directive is used.
// when including standard C++ libraries, you use angle brackets. When including your own files, you use quotes.
//https://en.cppreference.com/w/cpp/preprocessor/include
#include "main.h"
#include "rev/rev.hh"
#include "globals.hh"

// this makes it so you can just put the name of the item rather then putting rev:: before everything
using namespace rev;

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	// creates a skid steer chassis object, which is a chassis with a group of motors on the left and the right
	chassis = std::make_shared<rev::SkidSteerChassis>(left_motor_group, right_motor_group);

	// creates the odometry system. This is how the robot tracks its global position on the field. 
	// the parameters for it are defined in the globals.hh file
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

	// these are the background threads that update reckless, odometry, and the turn controller. 
    odom_runner = std::make_shared<rev::AsyncRunner>(odom);
	reckless_runner = std::make_shared<rev::AsyncRunner>(reckless);
	turn_runner = std::make_shared<rev::AsyncRunner> (turn);

	// initialize the lcd screen
	pros::lcd::initialize();

	// delay 2 seconds so that the IMU has time to initialize
  pros::delay(2000);

  // sets the robot's position to be (0_in, 0_in, 0_deg). This is probably not necessary 
  odom->reset_position();
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
	/*
	 * The code in this function should drive the robot in a 20in by 20in square.
	*/


	/*
	 * The following function call will drive the robot to the point 20_in, 0_in. The 0_deg is meaningless,
	 * but it is still required otherwise there will be a compiler error. 
	 * The code can be copy and pasted, you just have to change out the coordinates to go to a different spot. 
	*/
	reckless->go(
		RecklessPath().with_segment(
		RecklessPathSegment(
			std::make_shared<ConstantMotion>(0.5),             // tells the robot to move at 50% power
			std::make_shared<PilonsCorrection>(4, 0.3_in), // if the robot is 0.3in or more off the path, then it will start correcting that path
			std::make_shared<SimpleStop>(0.03_s, 0.15_s, 0.3),   // robot will soft stop if it is 0.15 seconds from the finish, and hard stop when it is 0.03 seconds from the finish. Soft stop means that the speed is set to 30% power. Hard stop means that the brakes are applied
			{ 20_in, 0_in, 0_deg },               // the target global position. Position 0, 0 is where the robot starts. the 0_deg is meaningless but it has to be included for syntax reasons
			0_in)                                          // tells the robot to stop 0_in from the target
	));
	reckless->await(); // don't run the next code until the robot has reached the target

	// turn the robot 90 degrees with a max power of 70%
	turn->turn_to_target_absolute(0.7, 90_deg);
	turn->await();



	reckless->go(RecklessPath().with_segment(
		RecklessPathSegment(
			std::make_shared<ConstantMotion>(0.5),
			std::make_shared<PilonsCorrection>(4, 0.3_in),
			std::make_shared<SimpleStop>(0.03_s, 0.15_s, 0.3), 
			{ 20_in, 20_in, 0_deg }, 0_in)
	));
	reckless->await();


	turn->turn_to_target_absolute(0.7, 180_deg);
	turn->await();




	reckless->go(RecklessPath().with_segment(
		RecklessPathSegment(
			std::make_shared<ConstantMotion>(0.5),
			std::make_shared<PilonsCorrection>(4, 0.3_in),
			std::make_shared<SimpleStop>(0.03_s, 0.15_s, 0.3), 
			{ 0_in, 20_in, 0_deg }, 0_in)
	));
	reckless->await();


	turn->turn_to_target_absolute(0.7, 270_deg);
	turn->await();





	reckless->go(RecklessPath().with_segment(
		RecklessPathSegment(
			std::make_shared<ConstantMotion>(0.5),
			std::make_shared<PilonsCorrection>(4, 0.3_in),
			std::make_shared<SimpleStop>(0.03_s, 0.15_s, 0.3), 
			{ 0_in, 0_in, 0_deg }, 0_in)
	));
	reckless->await();

	turn->turn_to_target_absolute(0.7, 0_deg);
	turn->await();
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
	}
}
