#include "main.h"

// Global Constants
const int LEFT_FRONT_MOTOR = 11;
const int LEFT_BACK_MOTOR = 20;
const int RIGHT_FRONT_MOTOR = 1;
const int RIGHT_BACK_MOTOR = 10;
const int JOYSTICK_DEADZONE = 5;
const bool DEBUG = true;

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button()
{
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
	pros::lcd::initialize();
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
void autonomous() {}

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
void opcontrol()
{
	pros::Controller xController(pros::E_CONTROLLER_MASTER);

	okapi::Motor xLeftFrontMotor(LEFT_FRONT_MOTOR);
	okapi::Motor xLeftBackMotor(LEFT_BACK_MOTOR);
	okapi::Motor xRightFrontMotor(RIGHT_FRONT_MOTOR);
	okapi::Motor xRightBackMotor(RIGHT_BACK_MOTOR);

	xLeftFrontMotor.setReversed(true);
	xLeftBackMotor.setReversed(true);

	std::shared_ptr<okapi::ChassisController> xChassisController =
		okapi::ChassisControllerBuilder()
			.withMotors(
				xLeftFrontMotor,  // Top left
				xRightFrontMotor, // Top right
				xRightBackMotor,  // Bottom right
				xLeftBackMotor	  // Bottom left
				)
			// Green gearset, 3.25 inch wheel diameter, 14 inch wheelbase
			.withDimensions(okapi::AbstractMotor::gearset::green, {{3.25_in, 14_in}, okapi::imev5GreenTPR})
			.withOdometry()
			.buildOdometry();

	auto xDrivetrain = std::static_pointer_cast<okapi::XDriveModel>(xChassisController->getModel());

	for (;;)
	{
		// Get joystick values
		int8_t xForward = xController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int8_t xStrafe = xController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
		int8_t xRotate = xController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

// For Debugging
#ifdef DEBUG
		pros::lcd::print(1, "Forward: %d", xForward);
		pros::lcd::print(2, "Strafe: %d", xStrafe);
		pros::lcd::print(3, "Rotate: %d", xRotate);
#endif

		xDrivetrain->xArcade(xStrafe / double(127), xForward / double(127), xRotate / double(127));

		pros::delay(20);
	}
}
