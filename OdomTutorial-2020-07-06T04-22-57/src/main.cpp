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
// RightEncoder         rotation      8               
// HorizontalEncoder    rotation      4               
// Controller1          controller                    
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <iostream>

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

// double leftEnc = 0;
// double rightEnc = 0;
// double horizEnc = 0;
// double RobotRotation = 0;

//This is how many miliseconds every cycle (recommended is 15)
int milisecondStepTime = 15;



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

//constants GET THESE VALUES
#define tl 7.250 //distance from center to left tracking wheel 
#define tr 7.250 //distance from center to right tracking wheel
#define tb 7.750 //distance from center to horiz tracking wheel

#define PI 3.141592653589793238462643383279502884
#define WHDIA 3.25 //wheel diameter

double getDegrees(double curRadian){
  return (curRadian * 180 / PI);
}

double posX = 0;
double posY = 0;
double theta = 0; //prev theta
double turnsL = 0; //previous position
double turnsR = 0;
double turnsB = 0;

void odometry(){
  // leftEnc = -LeftEncoder.position(degrees);
  // rightEnc = RightEncoder.position(degrees);
  // horizEnc = -HorizontalEncoder.position(degrees);
  // RobotRotation = InternalSensor.rotation();

  double ld = 0; //arc distance traveled by left tracking wheel, approx into line
  double rd = 0;
  double bd = 0;
  double dtheta = 0; //angle of arc in big circle in radians
  // double dist = 0;
  double ROTl = LeftEncoder.position(turns); //left wheel rotational sensor value
  double ROTr = RightEncoder.position(turns);
  double ROTb = HorizontalEncoder.position(turns);

  double currX = 0;
  double currY = 0; //current change in y coordinate

  ROTl -= turnsL; //get change in rotation
  ROTr -= turnsR;
  ROTb -= turnsB;


  ld = PI*ROTl*WHDIA; //calculate distance traveled by left tracking wheel
  rd = PI*ROTr*WHDIA;
  bd = PI*ROTb*WHDIA;

  dtheta = (rd - ld)/(tl + tr); //radians
  //btheta = bd/ //???? this is for the third encoder

  //rd/dtheta // radius of arc of right wheel
  double bigRadiusY = rd/dtheta + tr; // radius of big circle
  //double bigRadiusX = bd/dtheta + tb;

  // dist = (ld + rd)/2;
  //shift y-axis!

  //currX = 2*bigRadius*cos(theta+dtheta/2); 

  currY = 2*bigRadiusY*sin(dtheta/2); //law of cosines + half angle formula

  //printf("Rotation sensor -> %f, %f, %f, %f, %f\n", ROTl, ROTr, currX, currY, dtheta);
  
  //update global positions
  posX += currX;
  posY += currY;
  theta += dtheta;
  //keep angle orientation between -2pi and 2pi
  if (theta > 2*PI)
    theta -= (2*PI);

  if (theta < -(2*PI))
    theta += (2*PI);

  //update previous position
  turnsR += ROTr;
  turnsL += ROTl;
  turnsB += ROTb;
  
    Brain.Screen.clearScreen();

  Brain.Screen.setCursor(1, 1); 
  Brain.Screen.print(currY);

  Brain.Screen.setCursor(3, 1);
  Brain.Screen.print(posY);
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
    // leftEnc = -LeftEncoder.position(degrees);
    // rightEnc = RightEncoder.position(degrees);
    // horizEnc = -HorizontalEncoder.position(degrees);
    // RobotRotation = InternalSensor.rotation();

    odometry();

    //here I am printing values to the brain screen so I can see values of the encoders
    //Brain.Screen.clearScreen();

    // Brain.Screen.setCursor(1, 1); 
    // Brain.Screen.print(posX);
    // Brain.Screen.print("left: ");
    // Brain.Screen.setCursor(1, 15); 
    // Brain.Screen.print(leftEnc);

    // Brain.Screen.setCursor(2, 1); 
    // Brain.Screen.print("right: ");
    // Brain.Screen.setCursor(2, 15); 
    // Brain.Screen.print(rightEnc);

    // Brain.Screen.setCursor(3, 1); 
    // Brain.Screen.print("horiz: ");
    // Brain.Screen.setCursor(3, 15); 
    // Brain.Screen.print(horizEnc);
    
    
    // Brain.Screen.setCursor(4, 1); 
    // Brain.Screen.print("inertial: ");
    // Brain.Screen.setCursor(4, 15);  
    // Brain.Screen.print(RobotRotation);
    // wait(100, msec);
    
    // Brain.Screen.setCursor(5, 1); 
    // Brain.Screen.print("lastVerticalPosLocal: ");
    // Brain.Screen.setCursor(5, 15);  
    // Brain.Screen.print(lastVerticalPosLocal);
    // wait(100, msec);

    // Brain.Screen.setCursor(6, 1); 
    // Brain.Screen.print("lastHorizontalPosLocal: ");
    // Brain.Screen.setCursor(6, 15);  
    // Brain.Screen.print(lastHorizontalPosLocal);
    // wait(100, msec);

    // Brain.Screen.setCursor(7, 1); 
    // Brain.Screen.print("XPos: ");
    // Brain.Screen.setCursor(7, 15);  
    // Brain.Screen.print(XPos);
    // wait(100, msec);
    
    // Brain.Screen.setCursor(8, 1); 
    // Brain.Screen.print("YPos ");
    // Brain.Screen.setCursor(8, 15);  
    // Brain.Screen.print(YPos);
    // wait(100, msec);

    wait(99, msec); //I added this temporarily to be able to read the XPos and YPos before they update

    // if(Controller1.ButtonL1.pressing()){

    // }
  }
}
