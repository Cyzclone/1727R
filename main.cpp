#include "main.h"
#include "lemlib/api.hpp"
#include "pros/misc.h"


// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// drivetrain declarations
// negative port is backwards, _06 is 600rpm carts 
pros::Motor lF(-18, pros::E_MOTOR_GEARSET_06);  //left front motor
pros::Motor lM(-19, pros::E_MOTOR_GEARSET_06);  //left middle motor
pros::Motor lB(20, pros::E_MOTOR_GEARSET_06);   //left back motor
pros::Motor rF(8, pros::E_MOTOR_GEARSET_06);    //right front motor
pros::Motor rM(9, pros::E_MOTOR_GEARSET_06);    //right middle motor
pros::Motor rB(-10, pros::E_MOTOR_GEARSET_06);   //right back motor

pros::MotorGroup leftMotors({lF, lM, lB});  // left motor group
pros::MotorGroup rightMotors({rF, rM, rB}); // right motor group
pros::MotorGroup lift();


//other stuff
  pros::Motor intake(11);
  pros::ADIDigitalOut leftWing('H');
  pros::ADIDigitalOut rightWing('A');
  pros::ADIDigitalOut PTO('C');
 

lemlib::Drivetrain drivetrain{
    &leftMotors,                // left drivetrain motors
    &rightMotors,               // right drivetrain motors
    10,                         // 10 inch track width
    lemlib::Omniwheel::NEW_275, // using new 2.75" omnis
    450,                        // drivetrain rpm is 360
    8                           // chase power is 2. If we had traction wheels, it would have been 8
};

// inertial sensor
pros::Imu inertial_sensor(21); // port 2
                              // lateral motion controller
lemlib::ControllerSettings linearController(
    13,  // proportional gain (kP)
    20,  // derivative gain (kD)
    1,   // small error range, in inches
    100, // small error range timeout, in milliseconds
    3,   // large error range, in inches
    500, // large error range timeout, in milliseconds
    20   // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(
  3,   // proportional gain (kP)
                                             16,  // derivative gain (kD)
                                             1,   // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3,   // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             20   // maximum acceleration (slew)
);

// sensors for odometry
// note that in this example we use internal motor encoders, so we don't pass vertical tracking wheels
lemlib::OdomSensors sensors(nullptr,         // vertical tracking wheel 1, set to nullptr as we don't have one
                            nullptr,         // vertical tracking wheel 2, set to nullptr as we don't have one
                            nullptr,         // horizontal tracking wheel 1
                            nullptr,         // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &inertial_sensor // inertial sensor
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors);

void on_center_button()
{
  static bool pressed = false;
  pressed = !pressed;
  if (pressed)
  {
    pros::lcd::set_text(2, "I was pressed!");
  }
  else
  {
    pros::lcd::clear_line(2);
  }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
  pros::lcd::initialize();              // initialize brain screen
  chassis.calibrate();                  // calibrate sensors
               // X: 0, Y: 0, Heading: 0
   // X: 5.2, Y: 10.333, Heading: 87
                                        // the default rate is 50. however, if you need to change the rate, you
                                        // can do the following.
                                        // lemlib::bufferedStdout().setRate(...);
                                        // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

  // for more information on how the formatting for the loggers
  // works, refer to the fmtlib docs

  // thread to for brain screen and position logging
  pros::Task screenTask([&]()
                        {
        lemlib::Pose pose(0, 60, 0);
     
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        } });
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


void autonomous()
{
  chassis.setPose(0, 0, 180);
  
  


 
  



 
  
  
  
  
     
  // move to the point (53, 53) with a timeout of 1000 ms
                                   // chassis.turnTo(0, 12, 3000, true, 60);
  // chassis.moveTo(0, 12, 0,3000); // move to the point (53, 53) with a timeout of 1000 ms
  // chassis.moveTo(0, 0, 1000,75);
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
void macro()
{
   
}
void opcontrol()
{
  
  // declare toggle booleans
  bool arm = false;
  bool intakeF = false;
  bool intakeR = false;
  bool wingL = false;
  bool wingR = false;
 pros::Controller master(pros::E_CONTROLLER_MASTER);

    pros::MotorGroup leftMotors({lF, lM});
  while (true)
  {
    //arcade drive
    chassis.arcade(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X), 7);
 
    //intake toggle forward
     if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2))
    {
     
        if(intakeF == true)
        {
          intakeF = false;
        }
        intakeR = !intakeR;
     
    }

//intake toggle reverse
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2))
    {  
     
      if(intakeR == true)
        {
          intakeR = false;
        }
        intakeF = !intakeF;
    }

// adjust the actual speed
    if (intakeF)
    {
      intake = -127;
    }
    if (intakeR )
    {
     
      intake = 127;
    }
    if(!intakeR && !intakeF)
    {
      intake = 0;
    }


   


// wings
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1))
    {
     wingL = !wingL;
     
    }
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1))
    {
      wingR = !wingR;
    }

    if(wingR)
    {
      rightWing.set_value(true);
      
    }
    else{
       rightWing.set_value(false);
   
    }


if(wingL)
    {
      leftWing.set_value(true);
      
    }
    else{
       leftWing.set_value(false);
     
    }


    


    



    

   



   
   
  }
  }







