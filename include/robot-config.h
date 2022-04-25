using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor RightMotor1;
extern motor RightMotorREV;
extern motor RightMotor2;
extern motor LeftMotor1;
extern motor LeftMotorREV;
extern motor LeftMotor2;
extern controller Controller1;
extern controller Controller2;
extern inertial Inertial;
extern digital_out GoalCover;
extern digital_out RightPiston;
extern digital_out Clamp;
extern motor RingMech;
extern motor FourBar;
extern encoder EncoderG;
extern encoder EncoderE;
extern inertial Inertial;
extern motor_group LeftDriveSmart;
extern motor_group RightDriveSmart;
extern smartdrive DriveTrain;
/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );