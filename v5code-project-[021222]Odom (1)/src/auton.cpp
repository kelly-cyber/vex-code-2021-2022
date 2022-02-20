#include "vex.h"
#include "auton.h"
#include "MovementFunc.h"
#include <iostream>

void autonSkills(){
  //pistons out
  DigitalOutH.set(false);
  //backward to push red goal
  moveFwdRev(100, -2100);
  
  //turn right
  TurnInPlace(90, 7, 8); //if odom is running this doesn't stop
  //backward to wall
  //moveFwdRev(100, -100);
  //forward a bit
  moveFwdRev(100, 200);
  //turn left
  TurnInPlace(120, 7, 1); 

  //problems: Turn in place waivers back and forth
  //depends on speed which it shouldn't
  //need to fix turning 360 impact on odom global x, y. Offset
}