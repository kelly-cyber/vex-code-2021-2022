#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
inertial InternalSensor = inertial(PORT21);
rotation LeftEncoder = rotation(PORT9, false);
rotation RightEncoder = rotation(PORT8, false);
rotation HorizontalEncoder = rotation(PORT14, false);
controller Controller1 = controller(primary);
motor RightDriveMotorsMotorA = motor(PORT1, ratio18_1, true);
motor RightDriveMotorsMotorB = motor(PORT2, ratio18_1, false);
motor_group RightDriveMotors = motor_group(RightDriveMotorsMotorA, RightDriveMotorsMotorB);
motor LeftDriveMotorsMotorA = motor(PORT3, ratio18_1, false);
motor LeftDriveMotorsMotorB = motor(PORT4, ratio18_1, true);
motor_group LeftDriveMotors = motor_group(LeftDriveMotorsMotorA, LeftDriveMotorsMotorB);
motor Intake = motor(PORT19, ratio18_1, true);
motor LiftMotorA = motor(PORT6, ratio18_1, false);
motor LiftMotorB = motor(PORT5, ratio18_1, true);
motor_group Lift = motor_group(LiftMotorA, LiftMotorB);
digital_out DigitalOutH = digital_out(Brain.ThreeWirePort.H);
motor BackLift = motor(PORT12, ratio18_1, false);
potV2 Pot = potV2(Brain.ThreeWirePort.F);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}