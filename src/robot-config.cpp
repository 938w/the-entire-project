#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;
//4, 6, 7, 10, 13, 16, 19
// VEXcode device constructors
motor RightMotor1 = motor(PORT7, ratio18_1, false);
motor RightMotorREV = motor(PORT10, ratio18_1, true);
motor RightMotor2 = motor(PORT6, ratio18_1, false);
motor LeftMotor1 = motor(PORT16, ratio18_1, true);
motor LeftMotorREV = motor(PORT13, ratio18_1, false);
motor LeftMotor2 = motor(PORT4, ratio18_1, true);
controller Controller1 = controller(primary);
controller Controller2 = controller(partner);
motor RingMech = motor(PORT19, ratio6_1, false);
digital_out GoalCover = digital_out(Brain.ThreeWirePort.D);
digital_out RightPiston = digital_out(Brain.ThreeWirePort.B);
digital_out Clamp = digital_out(Brain.ThreeWirePort.C);
motor FourBar = motor(PORT21, ratio36_1, false);
encoder EncoderG = encoder(Brain.ThreeWirePort.G);
encoder EncoderE = encoder(Brain.ThreeWirePort.E);
inertial Inertial = inertial(PORT20);
motor_group RightDriveSmart = motor_group(RightMotor1, RightMotorREV, RightMotor2);
motor_group LeftDriveSmart = motor_group(LeftMotor1, LeftMotorREV, LeftMotor2);
smartdrive DriveTrain = smartdrive(LeftDriveSmart, RightDriveSmart, Inertial, 320, 320, 130, distanceUnits::mm, 1.4);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  Brain.Screen.print("Device initialization...");
  Brain.Screen.setCursor(2, 1);
  // calibrate the drivetrain Inertial
  wait(200, msec);
  Inertial.calibrate();
  Brain.Screen.print("Calibrating Inertial for Drivetrain");
  // wait for the Inertial calibration process to finish
  while (Inertial.isCalibrating()) {
    wait(25, msec);
  }
  // reset the screen now that the calibration is complete
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1,1);
  wait(50, msec);
  Brain.Screen.clearScreen();
}