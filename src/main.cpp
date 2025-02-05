#include "main.h"
#include "lift.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
ASSET(autonn_txt);

pros::MotorGroup leftMotors({17, -9, -10}, pros::MotorGearset::blue);    
pros::MotorGroup rightMotors({-16, 7, 8}, pros::MotorGearset::blue); 
pros::Motor intake(-20);
pros::Motor lb(-19);
pros::MotorGroup liftMotors({-19}, pros::MotorGearset::red);
pros::adi::DigitalOut mogo('H');
pros::adi::DigitalOut doink('G');
lib::Lift lift(&liftMotors, 1, {4, 0, 10});

pros::Imu imu(18);
pros::Rotation horz(-6);
lemlib::TrackingWheel horizontal_tracking_wheel(&horz, lemlib::Omniwheel::NEW_2, -.3);

lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              11.5, // track width (inches)
                              lemlib::Omniwheel::NEW_275, // wheels
                              600, // drivetrain rpm
                              2 // horizontal drift is 2 (for now)
);

lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10000000, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              30, // derivative gain (kD)
                                              0, // anSti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                            0 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(4, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              30, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              2, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain,
                        lateral_controller,
                        angular_controller,
                        sensors,
                        &throttle_curve, 
                        &steer_curve
);

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    lb.tare_position();
    lift.startTask();

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
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
 // set position to x:0, y:0, heading:0
    chassis.setPose(47, -7, 0);
    // turn to face heading 90 with a very long timeout
    // chassis.turnToHeading(90, 100000);
    chassis.moveToPose(47, -7, 33, 1500, {.maxSpeed = 127});
   pros::delay(500);
   lift.setTarget(190);
   pros::delay(500);
  chassis.moveToPoint(-36, -12, 1500, {.forwards = false}, true);
pros::delay(500);
lift.setTarget(0);
chassis.follow(autonn_txt, 5, 500000, false);

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
    lb.tare_position();
    int lbState = 0;
    bool clampState = false;
    bool doinkState = false;
    double lockPos = 0;
    bool liftButtReleased = true;
	while (true) {
		pros::Controller controller(pros::E_CONTROLLER_MASTER);

        // get left y and right x positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        chassis.arcade(leftY, rightX);
        if (controller.get_digital(DIGITAL_L1)) {
            intake.move(127);
        } 
        else if (controller.get_digital(DIGITAL_L2)) {
            intake.move(-127);
        } 
        else {
            intake.move(0);
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            liftButtReleased = true;
            lift.setState(lib::LiftState::Recieve);
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            liftButtReleased = false;
            lift.setState(lib::LiftState::Manual);
            lift.setVoltage(-127);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            liftButtReleased = false;
            lift.setState(lib::LiftState::Manual);
            lift.setVoltage(127);
        }
        else if (!liftButtReleased) {
            liftButtReleased = true;
            lift.setTarget(lift.motors->get_position());
        }

        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
                clampState = !clampState;
            }
        mogo.set_value(clampState);

        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
                doinkState = !doinkState;
            }
        doink.set_value(doinkState);

        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
            // set position to x:0, y:0, heading:0
            chassis.setPose(0, 0, 0);
            // chassis.turnToHeading(90, 1500);
            chassis.moveToPose(0,24,0,1500,{});
        }

        // delay to save resources
        pros::delay(25);
    }
}