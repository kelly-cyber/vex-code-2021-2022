using namespace vex;

extern brain Brain;

// VEXcode devices
extern inertial InternalSensor;
extern rotation LeftEncoder;
extern rotation RightEncoder;
extern rotation HorizontalEncoder;
extern controller Controller1;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );