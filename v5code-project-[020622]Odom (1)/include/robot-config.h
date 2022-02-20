using namespace vex;

extern brain Brain;

// VEXcode devices
extern inertial InternalSensor;
extern rotation vEncoder;
extern rotation hEncoder;
extern controller Controller1;
extern motor_group RightDriveMotors;
extern motor_group LeftDriveMotors;
extern motor Intake;
extern digital_out DigitalOutH;
extern motor BackLift;
extern potV2 Pot;
extern motor RightDrive;
extern motor LeftDrive;
extern motor Lift;
extern digital_out BackPiston;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );