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
// LeftEncoder          rotation      7               
// RightEncoder         rotation      8               
// HorizontalEncoder    rotation      14              
// Controller1          controller                    
// RightDriveMotors     motor_group   9, 2            
// LeftDriveMotors      motor_group   3, 4            
// Intake               motor         6               
// DigitalOutE          digital_out   E               
// BackLift             motor         12              
// Pot                  potV2         F               
// RightDrive           motor         20              
// LeftDrive            motor         11              
// Lift                 motor         5               
// BackClaw1            digital_out   A               
// BackClaw2            digital_out   C               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "Util.h"
#include "MovementFunc.h"
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

// double leftEnc = 0;
// double rightEnc = 0;
// double horizEnc = 0;
// double RobotRotation = 0;

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

void autonomous(void) {

Lift.resetPosition();

  // ..........................................................................
  // Insert autonomous user code here.
  // .........................................................................


  //put front claw down
  // Lift.setStopping(coast);
  // Lift.spin(forward);                                                                                                                               


  DigitalOutE.set(false);

 
  moveFwdRev(100, 1100);

  unclampBackClaw();
  DigitalOutE.set(true);

  // //put front claw up
  // // Lift.setStopping(hold);
  // // Lift.spin(reverse);

  // //drive backward to base
  moveFwdRev(100, -600);
    DigitalOutE.set(false);
      moveFwdRev(100, -200);

  // DigitalOutH.set(false);

  // moveFwdRev(100, -200);

  


  TurnInPlace(-300, 6, 4); 

  timeFwdRev( -70, 1.5f);
  clampBackClaw();
    moveFwdRev(100, 300);

  TurnInPlace(-90, 7, 1); 
  //     DigitalOutH.set(true);
  //       moveFwdRev(100, -1100);


}

//constants GET THESE VALUES INCHES
#define tl 7.250 //distance from center to left tracking wheel 
#define tr 7.250 //distance from center to right tracking wheel
#define tb 7.750 //distance from center to horiz tracking wheel

#define PI 3.141592653589793238462643383279502884
#define WHDIA 3.25 //wheel diameter

double getDegrees(double curRadian){ //not needed???
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

  // posX = 0; //TRY DEFINING HERE INSTEAD OF IN MAIN!!
  // posY = 0;
  // theta = 0; //prev theta
  // turnsL = 0; //previous position
  // turnsR = 0;
  // turnsB = 0;


  double ld = 0; //arc distance traveled by left tracking wheel, approx into line
  double rd = 0;
  double bd = 0;
  double dtheta = 0; //angle of arc in big circle in radians
  // double dist = 0;

  //rotational sensor value
  double ROTl = LeftEncoder.position(turns); 
  double ROTr = RightEncoder.position(turns);
  double ROTb = HorizontalEncoder.position(turns);

  double currX = 0;
  double currY = 0; //current change in y coordinate

  ROTl -= turnsL; //get change in rotation
  ROTr -= turnsR;
  ROTb -= turnsB;

  //calculate distance traveled by each tracking wheel
  ld = PI*ROTl*WHDIA; //(delta L)
  rd = PI*ROTr*WHDIA; //(delta R)
  bd = PI*ROTb*WHDIA; //(delta S)

  dtheta = (rd - ld)/(tl + tr); //change in angle, in radians
  //btheta = bd/ //???? this is for the third tracking wheel, ignore 

  //rd/dtheta is the radius of arc of right wheel
  double bigRadiusY = rd/dtheta + tr; // radius of big circle

  //bd/dtheta is the radius of arc of back wheel
  double bigRadiusX = bd/dtheta + tb; // radius of big circle

  // dist = (ld + rd)/2;
  
  if(dtheta == 0) //aka ld = rd
  {
    currX = bd;
    currY = ld;
  } else 
  {
    //formula calculated with law of cosines and half angle formula
    currX = 2*bigRadiusX*sin(dtheta/2); //change in x coor (on x-axis)
    currY = 2*bigRadiusY*sin(dtheta/2); //change in y coor (on y-axis)
  }

  //STARTING FROM STEP 9 pg8 OF PILONS DOC
  double avgTheta = theta + dtheta/2;

  //convert from cartesian to polar
  double rPolar = pow(currX*currX + currY*currY, 0.5);
  double thetaPolar = atan(currY/currX);

  //calculate global offset as local offset rotated by -avgTheta
  //b/c axis were offset by theta/2 "forward" to make local y axis and theta/2 "right" to make local x axis --> earlier radius calculations easier.
  thetaPolar += -avgTheta;

  //convert from polar back to cartesian
  currX = rPolar * cos(thetaPolar);
  currY = rPolar * sin(thetaPolar);

  //printf("Rotation sensor -> %f, %f, %f, %f, %f\n", ROTl, ROTr, currX, currY, dtheta); //this breaks the vex code pro application idk why
  
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
  turnsR += ROTr;
  turnsL += ROTl;
  turnsB += ROTb;
  
  Brain.Screen.clearScreen();

  Brain.Screen.setCursor(1, 1); 
  Brain.Screen.print("posX: ");
  Brain.Screen.setCursor(1, 15); 
  Brain.Screen.print(posX);

  Brain.Screen.setCursor(2, 1); 
  Brain.Screen.print("posY: ");
  Brain.Screen.setCursor(2, 15); 
  Brain.Screen.print(posY);

  Brain.Screen.setCursor(3, 1); 
  Brain.Screen.print("currX: ");
  Brain.Screen.setCursor(3, 15);
  Brain.Screen.print(currX);

  Brain.Screen.setCursor(4, 1); 
  Brain.Screen.print("currY: ");
  Brain.Screen.setCursor(4, 15); 
  Brain.Screen.print(currY);
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

bool out = false;
bool outB = false;
void usercontrol(void) {
  // User control code here, inside the loop
  RightDriveMotors.spin(forward);
  LeftDriveMotors.spin(forward);
  RightDrive.spin(forward);
  LeftDrive.spin(forward);
  Lift.spin(forward);
Lift.setStopping(hold);
  BackLift.spin(forward);
  BackLift.setStopping(hold);
  Intake.spin(forward);

  Lift.resetPosition();

  while (1) {
    RightDrive.setVelocity(Controller1.Axis2.position(), percent);
    RightDriveMotors.setVelocity(Controller1.Axis2.position(), percent);
    LeftDriveMotors.setVelocity(Controller1.Axis3.position(), percent);
    LeftDrive.setVelocity(Controller1.Axis3.position(), percent);

    //buttonL1
    if(Controller1.ButtonR1.pressing()){
      Intake.setVelocity(100, percent);
    } else if(Controller1.ButtonR2.pressing()){
      Intake.setVelocity(-100, percent);
    } else{
      Intake.setVelocity(0, percent);
    }

    //buttonR1
    if(Controller1.ButtonL2.pressing() && fabs(Lift.position(degrees))>10){
      Lift.setVelocity(-(100-4000/Lift.position(degrees)), percent);
    } else if(Controller1.ButtonL1.pressing()&& fabs(Lift.position(degrees))<1100){
      Lift.setVelocity(100 - (Lift.position(degrees) -500)/(19), percent);
    } else{
      Lift.setVelocity(0, percent);
    }

    //buttonA
    if(Controller1.ButtonA.pressing() ){
      if(out){
        DigitalOutE.set(true);
        out = false;
      } else{
        DigitalOutE.set(false);
        out = true;
      }
    }
 

    //buttonB and X
        if(Controller1.ButtonB.pressing() || Controller1.ButtonX.pressing()){
      if(outB){
        BackClaw1.set(true);
                BackClaw2.set(true);
        outB = false;
      } else{
        BackClaw1.set(false);
                BackClaw2.set(false);
        outB = true;
      }
    }

       while(Controller1.ButtonA.pressing()||Controller1.ButtonB.pressing() || Controller1.ButtonX.pressing()){
      wait(5, msec);
    }

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print(Lift.position(degrees));



    wait(10, msec);
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
  //Threads();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    // FOR SOME REASON, GLOBAL VARIABLES MUST DEFINE HERE TO PRINT W VARIABLE NAME?? 
    // leftEnc = -LeftEncoder.position(degrees);
    // rightEnc = RightEncoder.position(degrees);
    // horizEnc = -HorizontalEncoder.position(degrees);
    // RobotRotation = InternalSensor.rotation();

    posX = 0; //TRY DEFINING HERE!!
    posY = 0;
    theta = 0; //prev theta
    turnsL = 0; //previous position
    turnsR = 0;
    turnsB = 0;

    Correct(); //sets inertial sensor
    //odometry(); //TEMPORARY

    //here I am printing values to the brain screen so I can see values of the encoders
    //Brain.Screen.clearScreen();
    
    
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
    
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1); 
    //Brain.Screen.print(Pot.angle());
    // Brain.Screen.print(InternalSensor.angle());
    Brain.Screen.print(inert);

    wait(10, msec); //recc wait time according to pilons doc 10

    // if(Controller1.ButtonL1.pressing()){

    // }
  }
}
