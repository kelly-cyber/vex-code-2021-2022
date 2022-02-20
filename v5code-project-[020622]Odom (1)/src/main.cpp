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
// InternalSensor       inertial      17              
// vEncoder             rotation      7               
// hEncoder             rotation      14              
// Controller1          controller                    
// RightDriveMotors     motor_group   9, 2            
// LeftDriveMotors      motor_group   3, 4            
// Intake               motor         19              
// DigitalOutH          digital_out   H               
// BackLift             motor         12              
// Pot                  potV2         F               
// RightDrive           motor         20              
// LeftDrive            motor         11              
// Lift                 motor         5               
// BackPiston           digital_out   B               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "Util.h"
#include "MovementFunc.h"
#include "COdometry.h"
#include "user.h"
#include <cmath>
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


//This is how many miliseconds every cycle (recommended is 15)
int milisecondStepTime = 15;


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
  Init(); // initializes Inertial sensor and variables
  

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
float inert;
double yTurnOffset; 
double xTurnOffset; 
double inertError; 
double inertTimeError = 0;

int intake(){ //thread
  
  while (true){
    Intake.setVelocity(70, percent);
    vex::task::sleep(20);
  }
  
  return(0);
}

void Threads(){
vex::thread t( intake ); //want to intake forever to see how many rings we can get
}

//constants GET THESE VALUES INCHES
#define tr 7.250 //distance from center to v tracking wheel
#define tb 7.750 //distance from center to horiz tracking wheel

#define PI 3.141592653589793238462643383279502884
#define WHDIA 3.25 //wheel diameter
#define RW 7.15

double getDegrees(double curRadian){ //not needed???
  return (curRadian * 180 / PI);
}

double posX = 0; 
double posY = 0;
double theta = 0; //prev theta
double turnsV = 0; //previous position
double turnsH = 0;

void odometry(){

  double vd = 0; //arc distance traveled by vert tracking wheel, approx into line
  double hd = 0;
  double dtheta = inert; //angle of arc in big circle in radians

  //rotational sensor value
  double ROTv = vEncoder.position(turns); 
  double ROTh = hEncoder.position(turns);

  double currX = 0;
  double currY = 0; //current change in y coordinate

  ROTv -= turnsV; //get change in rotation
  ROTh -= turnsH;

  //calculate distance traveled by each tracking wheel
  vd = PI*ROTv*WHDIA; //(delta L)
  hd = PI*ROTh*WHDIA; //(delta R)

  dtheta -= theta; //change in angle, in radians

  //vd/dtheta is the radius of arc of v wheel
  double bigRadiusY = vd/dtheta + tr; // radius of big circle, a and b in the law of cosines

  //hd/dtheta is the radius of arc of back wheel
  double bigRadiusX = hd/dtheta + tb; // radius of big circle
  
  if(dtheta == 0) //aka ld = rd
  {
    // currX = bd;
    // currY = ld;
  } else 
  {
    //formula calculated with law of cosines and half angle formula
    currY = 2*bigRadiusY*sin(dtheta/2); //change in y coor (on y-axis)
    currX = 2*bigRadiusX*sin(dtheta/2); //change in x coor (on y-axis)
  }

//   //STARTING FROM STEP 9 pg8 OF PILONS DOC
//   double avgTheta = theta + dtheta/2;

//   //convert from cartesian to polar
//   double rPolar = pow(currX*currX + currY*currY, 0.5);
//   double thetaPolar = atan(currY/currX);

//   //calculate global offset as local offset rotated by -avgTheta
//   //b/c axis were offset by theta/2 "forward" to make local y axis and theta/2 "right" to make local x axis --> earlier radius calculations easier.
//   thetaPolar += -avgTheta;

//   //convert from polar back to cartesian
//   currX = rPolar * cos(thetaPolar);
//   currY = rPolar * sin(thetaPolar);

//   //printf("Rotation sensor -> %f, %f, %f, %f, %f\n", ROTl, ROTr, currX, currY, dtheta); //this breaks the vex code pro application idk why
  Brain.Screen.clearScreen();

  Brain.Screen.setCursor(1, 1); 
  Brain.Screen.print("vd: ");
  Brain.Screen.setCursor(1, 15); 
  Brain.Screen.print(vd);

  Brain.Screen.setCursor(2, 1); 
  Brain.Screen.print("hd: ");
  Brain.Screen.setCursor(2, 15); 
  Brain.Screen.print(hd);

  Brain.Screen.setCursor(3, 1); 
  Brain.Screen.print("dtheta: ");
  Brain.Screen.setCursor(3, 15);
  Brain.Screen.print(dtheta);

  Brain.Screen.setCursor(4, 1); 
  Brain.Screen.print("currX: ");
  Brain.Screen.setCursor(4, 15);
  Brain.Screen.print(currX);

  Brain.Screen.setCursor(5, 1); 
  Brain.Screen.print("currY: ");
  Brain.Screen.setCursor(5, 15);
  Brain.Screen.print(currY);

  //calculate NEW absolute position
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
  turnsV += ROTv;
  turnsH += ROTh;
  
}

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // .........................................................................
  
  //Odometry O;

  // //pistons out
  // DigitalOutH.set(false);
  // //forward to yellow goal
  // moveFwdRev(100, 1030);
  // DigitalOutH.set(true);

  //drive backward
  // moveFwdRev(100, -800);
  // DigitalOutH.set(false);
  // moveFwdRev(100, -300);

  // //turn to tall yellow goal??
  // TurnInPlace(-38, 7, 1); 
  // moveFwdRev(100, 1300);
  // DigitalOutH.set(true);

  // //backward return
  // moveFwdRev(100, -700);

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
  userDriveFunc();
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
  //Threads();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    // FOR SOME REASON, GLOBAL VARIABLES MUST DEFINE HERE TO PRINT W VARIABLE NAME?? 
    // leftEnc = -LeftEncoder.position(degrees);
    // rightEnc = RightEncoder.position(degrees);
    // horizEnc = -HorizontalEncoder.position(degrees);
    // RobotRotation = InternalSensor.rotation();

    // posX = 0; //TRY DEFINING HERE!!
    // posY = 0;
    // theta = 0; //prev theta
    // turnsL = 0; //previous position
    // turnsR = 0;
    // turnsB = 0;

    Correct(); //sets inertial sensor
    //odometry(); //TEMPORARY


    Brain.Screen.clearScreen();
    // Brain.Screen.setCursor(1, 1); 
    // //Brain.Screen.print(Pot.angle());
    // // Brain.Screen.print(InternalSensor.angle());
    // Brain.Screen.print("Robot Heading: ");
    // Brain.Screen.print(inert);

    odometry();

    wait(80, msec); //recc wait time according to pilons doc 10
  }
}
