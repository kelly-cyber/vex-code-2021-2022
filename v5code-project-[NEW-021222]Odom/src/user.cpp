#include "vex.h"
#include "user.h"
#include <iostream>

bool out = false;
bool outB = false;
void userDriveFunc(){
  RightDriveMotors.spin(forward);
  LeftDriveMotors.spin(forward);
  RightDrive.spin(forward);
  LeftDrive.spin(forward);
  Lift.spin(forward);
  Lift.setStopping(hold);
  BackLift.spin(forward);
  BackLift.setStopping(hold);
  Intake.spin(forward);

  while (1) {
    RightDrive.setVelocity(Controller1.Axis2.position(), percent);
    RightDriveMotors.setVelocity(Controller1.Axis2.position(), percent);
    LeftDriveMotors.setVelocity(Controller1.Axis3.position(), percent);
    LeftDrive.setVelocity(Controller1.Axis3.position(), percent);

    //buttonL1
    if(Controller1.ButtonL1.pressing()){
      Intake.setVelocity(-70, percent);
    } else if(Controller1.ButtonL2.pressing()){
      Intake.setVelocity(70, percent);
    } else{
      Intake.setVelocity(0, percent);
    }

    //buttonR1
    if(Controller1.ButtonR2.pressing()){
      Lift.setVelocity(-100, percent);
    } else if(Controller1.ButtonR1.pressing()){
      Lift.setVelocity(100, percent);
    } else{
      Lift.setVelocity(0, percent);
    }

    //buttonA
    if(Controller1.ButtonA.pressing() ){
      if(out){
        DigitalOutH.set(true);
        out = false;
      } else{
        DigitalOutH.set(false);
        out = true;
      }
    }
 

    //buttonB and X
        if(Controller1.ButtonB.pressing() || Controller1.ButtonX.pressing()){
      if(outB){
        BackPiston.set(true);
        outB = false;
      } else{
        BackPiston.set(false);
        outB = true;
      }
    }

       while(Controller1.ButtonA.pressing()||Controller1.ButtonB.pressing() || Controller1.ButtonX.pressing()){
      wait(5, msec);
    }



    wait(5, msec);
  }
}