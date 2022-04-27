/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "pid.h"
#include "vex.h"
using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
bool state = false;
bool last = false;
bool boxstate = false;
bool boxlast = false;
bool ringstate = false;
bool ringlast = false;
bool pistonOn = false;

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}
int myThread() {
  int count = 0;
  while (1) {
    int notmovingcount = 0;
    double lastpos = RingMech.position(degrees);
    vex::this_thread::sleep_for(25);
    if (lastpos - RingMech.position(deg) < 1) {
      notmovingcount++;
    }
    if (notmovingcount >= 10) {
      Controller1.Screen.print("runnintask");
      RingMech.spin(reverse);
      vex::this_thread::sleep_for(25);
      RingMech.stop();
      RingMech.spin(forward);
    }
  }
  return (0);
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
  // a password
  // please see route.txt for the route details
  pid2 PID;
  FourBar.stop(hold);
  RightPiston.set(true);
  Clamp.set(true);
  FourBar.setVelocity(100, pct);
  // vex::thread t(myThread);
  DriveTrain.setTurnVelocity(70, percent);

  // goal 1
  PID.drive2(20.5);
  DriveTrain.turnToHeading(-90, degrees);
  PID.drive2(-16, 10000);
  RightPiston.set(false);
  PID.drive2(16.5, 10000);

  // mobile goal 1
  DriveTrain.turnToHeading(0, degrees);
  wait(0.2, sec);
  PID.drive2(25.5, 10000);
  Clamp.set(false);

  // raise 4bar a bit
  FourBar.spinFor(forward, 580, deg, 100, rpm, false);

  // turn to platfrom
  DriveTrain.setTurnVelocity(30, percent);
  DriveTrain.turnToHeading(-18, degrees);
  wait(0.4, sec);

  // run ring intake
  RingMech.setVelocity(100, pct);
  RingMech.spin(forward);

  // Drive forward to platform
  PID.drive2(33, 7000);

  // Turn a more to get second pile
  DriveTrain.turnToHeading(-90, degrees);
  PID.drive2(26, 7000);
  PID.drive2(-3, 7000);

  // Turn to platform
  DriveTrain.turnToHeading(0, degrees);

  // Drive to platform
  PID.drive2(16, 10000);

  // Lower Fourbar
  FourBar.spinFor(reverse, 210, degrees);

  // Release
  Clamp.set(true);
  wait(0.1, sec);

  // Backout
  FourBar.spinFor(forward, 70, degrees);
  wait(0.1, sec);
  FourBar.spinFor(reverse, 440, degrees, false);
  PID.drive2(-16, 10000);

  // Release Goal 1
  RightPiston.set(true);
  PID.drive2(10);

  // Turn to goal
  DriveTrain.setTurnVelocity(50, percent);
  DriveTrain.turnToHeading(-180, degrees);

  // Drive to goal
  DriveTrain.setDriveVelocity(40, percent);
  DriveTrain.driveFor(forward, 11, inches);

  // clamp
  Clamp.set(false);

  // Turn Around and Lift up Goal
  FourBar.spinFor(580, degrees, false);
  DriveTrain.setTurnVelocity(50, percent);
  DriveTrain.turnToHeading(-20, degrees);

  // Drive to platform
  PID.drive2(16, 7000);

  /*
  // Lower FourBar
  FourBar.spinFor(reverse, 190, degrees);

  //Release clamp
  Clamp.set(true);
  wait (0.1, sec);

  //Backout
  FourBar.spinFor(forward, 70, degrees);
  wait(0.1, sec);
  FourBar.spinFor(reverse, 440, degrees, false);
  PID.drive2(-16, 10000);
  */
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  RingMech.spin(forward, 600, rpm);
  RightPiston.set(true);
  Clamp.set(true);
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    RightMotor1.spin(forward, Controller1.Axis2.position(), percent);
    RightMotorREV.spin(forward, Controller1.Axis2.position(), percent);
    RightMotor2.spin(forward, Controller1.Axis2.position(), percent);
    LeftMotor1.spin(reverse, Controller1.Axis3.position(), percent);
    LeftMotorREV.spin(reverse, Controller1.Axis3.position(), percent);
    LeftMotor2.spin(reverse, Controller1.Axis3.position(), percent);

    if (Controller1.ButtonA.pressing() && !last) {
      last = true;
      state = !state;
    }
    if (!Controller1.ButtonA.pressing()) {
      last = false;
    }

    if (state) {
      if (Controller1.Axis3.position() < 7 &&
          Controller1.Axis3.position() > -7) {

        LeftMotor1.stop(hold);
        LeftMotor2.stop(hold);
        LeftMotorREV.stop(hold);
      }
      if (Controller1.Axis2.position() < 7 &&
          Controller1.Axis2.position() > -7) {
        RightMotor1.stop(hold);
        RightMotorREV.stop(hold);
        RightMotor2.stop(hold);
      }
    }
    if (!state) {
      if (Controller1.Axis3.position() < 7 &&
          Controller1.Axis3.position() > -7) {

        LeftMotor1.stop(coast);
        LeftMotor2.stop(coast);
        LeftMotorREV.stop(coast);
      }
      if (Controller1.Axis2.position() < 7 &&
          Controller1.Axis2.position() > -7) {
        RightMotor1.stop(coast);
        RightMotorREV.stop(coast);
        RightMotor2.stop(coast);
      }
    }
    // bbhbhbhbhbhbhvgvgvvgvghvgvgvgvghvvgvvghvghvghvgvgvhgvgvgvgvghvcgvgvgvgbhbhbbhbhbhbjbbhbhbbnbnmbnmbmnbmnbnbhjvggccgchcghcghcgcgcghcgcgcghcghcgcghcghcgcghcgcghchgcgcghchccggcghccgcfccfchcfcgfcchccffffgfjhfgjfgjfjfjffhgjfgjfghfhgffgfgfgfhgftftftfthfgffyftftfthfjfgfjghfghfghjfghjfghfgjfghjfhgjfgfghfjfgfgfgffjfhgjfgjfgjfgjfgfgfjfgffgjfffffghjfgfgfffjgfhgfgjfgjfgfjtyfgvhgftfvgvtftvbgvtgugtftgygygygyyygggvcfcvfcfcfcfcfcfgvfvgcvvfvfcfcgvfxdcbhvfcghhgvtfvbbjfwghehahclegrhfgrjvhrhvvggbhgghghghghghghhhhhghghgvrhbrhgnvnbbjtthrbfhghrbvbfbcbfhr
    // ghrhgfbbffhbfhfbjnjn
    Controller2.Screen.newLine();
    Controller2.Screen.print(Controller1.ButtonR1.pressing());
    if ((Controller1.ButtonRight.pressing() ||
         Controller2.ButtonR1.pressing()) &&
        !ringlast) {
      ringlast = true;
      ringstate = !ringstate;
    }

    if (!Controller1.ButtonRight.pressing() ||
        !Controller2.ButtonR1.pressing()) {
      ringlast = false;
    }
    if (ringstate) {
      if (Controller1.ButtonDown.pressing() ||
          Controller2.ButtonR2.pressing()) {
        RingMech.spin(reverse, 600, rpm);

      } else {
        RingMech.spin(forward, 600, rpm);
      }
    }
    if (!ringstate) {
      RingMech.stop();
    }
    if (Controller1.ButtonR2.pressing()) {

      RightPiston.set(true);
    }
    if (Controller1.ButtonR1.pressing()) {

      RightPiston.set(false);
    }
    if (Controller1.ButtonY.pressing()) {
      Clamp.set(true);
    }
    if (Controller1.ButtonB.pressing()) {
      Clamp.set(false);
    }
    if (Controller1.ButtonL1.pressing()) {
      FourBar.spin(forward, 100, rpm);
    }
    if (Controller1.ButtonL2.pressing()) {
      FourBar.spin(reverse, 100, rpm);
    }
    if (!Controller1.ButtonL2.pressing() && !Controller1.ButtonL1.pressing()) {
      FourBar.stop(hold);
    }
    if (Controller1.ButtonX.pressing() && !boxlast) {
      boxlast = true;
      boxstate = !boxstate;
    }
    if (!Controller1.ButtonX.pressing()) {
      boxlast = false;
    }
    if (boxstate) {
      GoalCover.set(true);
    }
    if (!boxstate) {
      GoalCover.set(false);
    }
    wait(5, msec);
    // Sleep the task for a short amount of time to
    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
