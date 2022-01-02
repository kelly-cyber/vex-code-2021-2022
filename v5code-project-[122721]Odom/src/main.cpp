/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// InternalSensor       inertial      1               
// LeftEncoder          rotation      2               
// RightEncoder         rotation      3               
// HorizontalEncoder    rotation      4               
// Controller1          controller    20                
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;



/////////////////////////////////////////////////////////////////////////
//SETTINGS
/////////////////////////////////////////////////////////////////////////

//Utilize these three variables to make sure they are reading correctly.
//The horizontal encoder should be positive if you push the robot laterally to the right
//The vertical encoders should be positive if you push the robot laterally forward
//Rotating the robot clockwise should yield a positive degree value
//NOTE: The recommended position of the vertical encoders(left and right) should be located at center of mass
// int leftEnc = -LeftEncoder.position(degrees);
// int rightEnc = RightEncoder.position(degrees);
// int horizEnc = HorizontalEncoder.position(degrees);
// double RobotRotation = InternalSensor.rotation();

double leftEnc = 0;
double rightEnc = 0;
double horizEnc = 0;
double RobotRotation = 0;

//This is how many miliseconds every cycle (recommended is 15)
int milisecondStepTime = 15;

//This is something you should utilize to convert the horizontal encoder distance to match the vertical encoder distance
//Grab a ruler, and push the robot 20 inches straight forward, then record the average vertical value (Example: 51269)
//Reset, Grab a ruler, and push the robot 20 inches to the right, then record the horizontal encoder value (Example: 10532)
//You will need to convert it such that the horizontal matches for the same distance, so the multiplier will be horizontal(10532)/vertical(51269)
//And that number will be the ratio (10532/51269 = 0.2054262809885116)
double verticalToHorizontalRatio = 1.0000000; 

/* L: 814.92 R: 813.78    avg:814.35
L: 821.69  R:823.97       avg:822.83
L: 825.65  R:820.37       avg: 823.01
H: 699.96 */ //RE-DO HORIZONTAL w elastics

//Because we can average the vertical encoders while turning, it will not show a change
//But for the horizontal encoders, it will assume the robot is laterally strafing, which is not correct
//We fix this by applying a multiplier compensation such that we will negate it using the inertial sensor
//Reset your robot, make sure everything is calibrated
//Rotate your robot such that the inertial sensor reads 3600 degrees (i.e. rotate the robot 10 times for the sake of accuracy)
//Your horizontal encoder should have a value (Example: 231809)
//Divide that result by 3600 (231809/3600 = 64.3913888), and that number will be the value used
double rotateNegateHorizontalRatio = 50.000000;

//This is utilized to convert the ticks moved to actual inches
//Push the robot 20 inches forward (without turning it or pushing it strafe-wise)
//Get the average reading of the vertical encoders (Example: 253053)
//Divide the reading by 20 (253053/20 = 12,652.65), and put the multipler in this area
//This will convert it to inches
double posTranslationMultiplier = 41.1505;
/////////////////////////////////////////////////////////////////////////







/////////////////////////////////////////////////////////////////////////
//Inclusions added manually
/////////////////////////////////////////////////////////////////////////
// Include additional mathematical operations
#include <cmath>
/////////////////////////////////////////////////////////////////////////




/////////////////////////////////////////////////////////////////////////
//Functions added manually
/////////////////////////////////////////////////////////////////////////
// Waiting function that returns time actually waited (Credit to James Pearman (jpearman on VEXForum - https://www.vexforum.com/t/vexcode-sleep-help/82706/2?u=connor))
uint32_t wait( uint32_t time_mS ) {
    uint64_t start = Brain.Timer.systemHighResolution();
    this_thread::sleep_for(time_mS);
    return (uint32_t)(Brain.Timer.systemHighResolution() - start); 
}
// This converts degrees to radians
double getRadians(int degrees){
  return ( degrees * M_PI ) / 180 ;
}
/////////////////////////////////////////////////////////////////////////




/////////////////////////////////////////////////////////////////////////
//Initialized variables for usage
/////////////////////////////////////////////////////////////////////////
// Include additional mathematical operations
double lastVerticalPosLocal = 0.0;
double lastHorizontalPosLocal = 0.0;
double XPos = 0.0;
double YPos = 0.0;
/////////////////////////////////////////////////////////////////////////



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
  while (1) {

    // Wait an amount of seconds, while returning the actual wait time to the variable "m" 
    //When telling a computer to wait, it may have a load-amount that would result in it yielding earlier or later then the actual time, that's why we do this
    int m = wait(milisecondStepTime);

    /////////////////////////////////////////////////////////////////////////
    //Does quicc maths to find out an accurate encoder readings of local position change
    /////////////////////////////////////////////////////////////////////////
    //This is for the local position of the robot (THIS IS NOT WORLD POSITION)
    double VerticalPosLocal = (leftEnc + rightEnc) / 2.0;
    double HorizontalPosLocal = horizEnc + (RobotRotation * rotateNegateHorizontalRatio);
    /////////////////////////////////////////////////////////////////////////



    //Applies ratio
    HorizontalPosLocal *= verticalToHorizontalRatio;

    //By getting position - lastposition within 20 miliseconds, this will give us velocity
    //Similar to PID alrogithms, this converts position -> Velocity by gettings its derivative through the continuous loop
    double YV = VerticalPosLocal - lastVerticalPosLocal;
    double XV = HorizontalPosLocal - lastHorizontalPosLocal;

    //Do some relatively-complex maths that converts local velocity to world-space velocity. The (m/milisecondStepTime) at the end is a multiplier 
    //that would scale velocity more appropriately due to computer yields (Line 131 explains) If computer is told to wait 2 second and waits 0.75 instead,
    //the multiplier would then scale the velocity to be 0.375 its value (0.75/2 = 0.375)
    double XVelocityWorldSpace = (XV * std::cos(getRadians(-RobotRotation)) - YV * std::sin(getRadians(-RobotRotation))) * (m/milisecondStepTime);
	  double YVelocityWorldSpace = (XV * std::sin(getRadians(RobotRotation)) - YV * std::cos(getRadians(RobotRotation))) * (m/milisecondStepTime);

    //This would give us the X and Y position in world space
    XPos += XVelocityWorldSpace * posTranslationMultiplier;
    YPos += YVelocityWorldSpace * posTranslationMultiplier;

    //This is applied to be utilized 20 miliseconds after
    lastVerticalPosLocal = VerticalPosLocal;
    lastHorizontalPosLocal = HorizontalPosLocal;

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
    // FOR SOME REASON, GLOBAL VARIABLES MUST DEFINE HERE TO PRINT W VARIABLE NAME?? 
    leftEnc = -LeftEncoder.position(degrees);
    rightEnc = RightEncoder.position(degrees);
    horizEnc = -HorizontalEncoder.position(degrees);
    RobotRotation = InternalSensor.rotation();

    //here I am printing values to the brain screen so I can see values of the encoders
    Brain.Screen.clearScreen();

    Brain.Screen.setCursor(1, 1); 
    Brain.Screen.print("left: ");
    Brain.Screen.setCursor(1, 15); 
    Brain.Screen.print(leftEnc);

    Brain.Screen.setCursor(2, 1); 
    Brain.Screen.print("right: ");
    Brain.Screen.setCursor(2, 15); 
    Brain.Screen.print(rightEnc);

    Brain.Screen.setCursor(3, 1); 
    Brain.Screen.print("horiz: ");
    Brain.Screen.setCursor(3, 15); 
    Brain.Screen.print(horizEnc);
    
    
    Brain.Screen.setCursor(4, 1); 
    Brain.Screen.print("inertial: ");
    Brain.Screen.setCursor(4, 15);  
    Brain.Screen.print(RobotRotation);
    wait(100, msec);
    
    Brain.Screen.setCursor(5, 1); 
    Brain.Screen.print("lastVerticalPosLocal: ");
    Brain.Screen.setCursor(5, 15);  
    Brain.Screen.print(lastVerticalPosLocal);
    wait(100, msec);

    Brain.Screen.setCursor(6, 1); 
    Brain.Screen.print("lastHorizontalPosLocal: ");
    Brain.Screen.setCursor(6, 15);  
    Brain.Screen.print(lastHorizontalPosLocal);
    wait(100, msec);

    Brain.Screen.setCursor(7, 1); 
    Brain.Screen.print("XPos: ");
    Brain.Screen.setCursor(7, 15);  
    Brain.Screen.print(XPos);
    wait(100, msec);
    
    Brain.Screen.setCursor(8, 1); 
    Brain.Screen.print("YPos ");
    Brain.Screen.setCursor(8, 15);  
    Brain.Screen.print(YPos);
    wait(100, msec);

    wait(999, msec); //I added this temporarily to be able to read the XPos and YPos before they update

    // if(Controller1.ButtonL1.pressing()){

    // }
  }
}
