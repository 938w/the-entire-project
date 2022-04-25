#include "pid.h"
#include <cmath>

using namespace std;

// get position from encoder
double getposition() {
  return ((EncoderE.rotation(rotationUnits::deg) * -1 +
           EncoderG.rotation(rotationUnits::deg)) /
          2) *
         M_PI * 2.75 / 360;
};

double getposition2() {
  return ((LeftDriveSmart.rotation(rotationUnits::deg) +
           RightDriveSmart.rotation(rotationUnits::deg)) /
          2) *
         M_PI * 4 * 1.4 / 360;
};

double signnum_c(double x) {
  if (x > 0.0)
    return 1.0;
  if (x < 0.0)
    return -1.0;
  return x;
};

void pid2::drive2(double target, double velocity, double dir) {
  Inertial.resetRotation();
  EncoderE.resetRotation();
  EncoderG.resetRotation();
  LeftDriveSmart.resetRotation();
  RightDriveSmart.resetRotation();

  // default to get yam from inertial
  double yaw = dir;
  if (dir == 999) {
    yaw = Inertial.yaw();
  }

  double minVelocity = 2000;
  int notMovingCount = 0;
  double lastPosition = 0;

  // going straight pid
  double error;
  double lasterror = 0;
  double derivative;
  double velocityFromTheError;
  double integral = 0;
  double p = 110;
  double i = 0;
  double d = 0.005;

  // distance pid
  double d_error;
  double d_lasterror = 0;
  double d_derivative;
  double d_velocityFromTheError;
  double d_integral = 0;
  double d_p = 450;
  double d_i = 0.2;
  double d_d = 20;

  // we love overshot a little here rather undershort or going back and forth to adjust
  while ((getposition() < target && target > 0) ||
         (getposition() > target && target < 0)) {
    double position = getposition();

    if (lastPosition == position) {
      notMovingCount++;
    }

    lastPosition = position;

    // if stuck for more than 1sec
    if (notMovingCount >= 100) {
      break;
    }

    // pid for driving straight
    error = Inertial.yaw() - yaw;
    derivative = error - lasterror;
    lasterror = error;
    integral += error;
    velocityFromTheError = (error * p + derivative * d + integral * i);

    // distance pid
    d_error = target - position;
    d_derivative = d_error - d_lasterror;
    d_lasterror = d_error;
    d_integral += d_error;

    // e.g. step1: error: 10in, starting speed will be 10*P
    // e.g. step2: error:8in, 8*p + (-2)*d + (18)*i
    d_velocityFromTheError =
        (d_error * d_p + d_derivative * d_d + d_integral * d_i);

    // max velocity
    if (abs(d_velocityFromTheError) > velocity) {
      d_velocityFromTheError = velocity * signnum_c(d_velocityFromTheError);
    }

    // min velocity
    if (abs(d_velocityFromTheError) < minVelocity) {
      d_velocityFromTheError = minVelocity * signnum_c(d_velocityFromTheError);
    }

    Controller1.Screen.newLine();
    Controller1.Screen.print(position);

    // using mV here, max = 12000
    LeftDriveSmart.spin(fwd, d_velocityFromTheError - velocityFromTheError,
                        voltageUnits::mV);
    RightDriveSmart.spin(fwd, d_velocityFromTheError + velocityFromTheError,
                         voltageUnits::mV);

    wait(10, msec);
  }

  LeftDriveSmart.stop(brakeType::brake);
  RightDriveSmart.stop(brakeType::brake);
};

// drive straight is PID but slow down is not
void pid2::drive(double target, double velocity, double slowDistance,
                 double dir) {
  Inertial.resetRotation();
  EncoderE.resetRotation();
  EncoderG.resetRotation();
  double yaw = dir;
  if (dir == 999) {
    yaw = Inertial.yaw();
  }
  double error;
  double lasterror = 0;
  double derivative;
  double velocityFromTheError;
  double integral = 0;
  double p = 1;
  double i = 0;
  double d = 0.005;
  double accelerationStep = 16;
  int notMovingCount = 0;
  double lastPosition = 0;

  if (target > 0) {

    // going forward
    while (getposition() < target) {

      double position = getposition();

      if (lastPosition == position) {
        notMovingCount++;
      }

      lastPosition = position;

      // if stuck for more than 1sec
      if (notMovingCount > 100) {
        break;
      }

      error = Inertial.yaw() - yaw;
      derivative = error - lasterror;
      lasterror = error;
      integral += error;
      velocityFromTheError = (error * p + derivative * d + integral * i);

      if (getposition() > target - slowDistance) {
        velocity = velocity - accelerationStep;
      }
      if (velocity < 10)
        velocity = 10;
      Controller1.Screen.print("-");
      Controller1.Screen.print(velocity);

      LeftDriveSmart.spin(fwd, velocity - velocityFromTheError, percent);
      RightDriveSmart.spin(fwd, velocity + velocityFromTheError, percent);

      wait(10, msec);
    }
  } else {
    // going backward
    while (getposition() > target) {

      double position = getposition();
      if (lastPosition == position) {
        notMovingCount++;
      }

      lastPosition = position;

      // if stuck for more than 1sec
      if (notMovingCount > 100) {
        break;
      }

      error = Inertial.yaw() - yaw;
      derivative = error - lasterror;
      lasterror = error;
      integral += error;
      velocityFromTheError = (error * p + derivative * d + integral * i);

      if (getposition() < target + slowDistance) {
        velocity = velocity - accelerationStep;
      }
      if (velocity < 10)
        velocity = 10;
      LeftDriveSmart.spin(reverse, velocity + velocityFromTheError, percent);
      RightDriveSmart.spin(reverse, velocity - velocityFromTheError, percent);

      wait(10, msec);
    }
  }

  LeftDriveSmart.stop(brakeType::brake);
  RightDriveSmart.stop(brakeType::brake);
};

void pid2::turnToHeading(double heading) {
  // min velocity
  double minVelocity = 2000;

  double error;
  double lasterror = 0;
  double derivative = 0;
  double velocityFromTheError;
  double integral = 0;

  double p = 100;
  double i = 0.0001;
  double d = 0.01;

  // offset 2 degree
  while (abs(heading - Inertial.heading()) >= 2) {
    error = heading - Inertial.heading();
    derivative = error - lasterror;
    lasterror = error;
    integral += error;

    // error range from -360 to 360, anything above 120, will start with full
    // speed: 12000mv
    //
    // e.g. step1: current heading: 0, target 90, error = 90,
    // e.g. step2: current heading: 10, v = 80*KP + (-10)*KD + (170)*KI
    velocityFromTheError = (error * p + derivative * d + integral * i);

    if (velocityFromTheError > 12000) {
      velocityFromTheError = 12000;
    }

    // min velocity
    if (velocityFromTheError < minVelocity) {
      velocityFromTheError = minVelocity;
    }

    LeftDriveSmart.spin(fwd, velocityFromTheError, voltageUnits::mV);
    RightDriveSmart.spin(reverse, velocityFromTheError, voltageUnits::mV);

    wait(10, msec);
  }

  LeftDriveSmart.stop(brakeType::brake);
  RightDriveSmart.stop(brakeType::brake);
};