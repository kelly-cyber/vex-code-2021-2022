#include "vex.h"
#include "MovementFunc.h"
#include "Util.h"

void StopMotorsEnd(){

  RightDriveMotors.stop(brake);
  RightDrive.stop(brake);
  LeftDriveMotors.stop(brake);
  LeftDrive.stop(brake);

}

void timeFwdRev(int motorPower, float sec){ //both neg to rev

  //set velocities for drivetrain motors
  RightDriveMotors.setVelocity(motorPower, pct);
  LeftDriveMotors.setVelocity(motorPower, pct);

  //int Degrees = inches/13*360; //convert inches to degrees

  //spin until reached seconds
  RightDriveMotors.spin(forward);
  LeftDriveMotors.spin(forward);
  
  wait(sec, seconds);

  StopMotorsEnd(); //instead of stopping motors, wait(delay in seconds, seconds)?

}


void tempTurn(int motorPower, float sec){

  RightDriveMotors.setVelocity(-motorPower ,pct);//right ones need to be reversed
  LeftDriveMotors.setVelocity(motorPower ,pct); 

  RightDriveMotors.spin(forward);
  LeftDriveMotors.spin(forward);  

  wait(sec, seconds);
  
  StopMotorsEnd();
}

// void frontLift(int motorPower, float sec){
//   Lift.setVelocity(motorPower, pct);
// }

void moveFwdRev(double motorPower, int Degrees){ //neg degrees to move rev

  LeftDrive.setPosition(0, degrees); //set y encoder to 0
  double KP = 1;
  motorPower = motorPower*0.01; //pct

  //set velocities for drivetrain motors
  RightDriveMotors.setVelocity(motorPower, pct);
  RightDrive.setVelocity(motorPower, pct);
  LeftDriveMotors.setVelocity(motorPower, pct);
  LeftDrive.setVelocity(motorPower, pct);

  //int Degrees = inches/13*360; //convert inches to degrees
  int Error = Degrees;

  RightDriveMotors.spin(forward);
  RightDrive.spin(forward);
  LeftDriveMotors.spin(forward);
  LeftDrive.spin(forward);

  while(Error != 0){
    Error = Degrees - LeftDrive.position(degrees); //(TR.position(degrees) + BR.position(degrees))/2 - (TL.position(degrees) + BL.position(degrees))/2; //estimate center by /2

    double powerDif = (3^Error)/4; //if we're 120 deg from target, speed is 20pct //Error/6 originally
    if(powerDif>100){
      powerDif=100;
    }
     if(powerDif<-100){
      powerDif=-100;
    }
    powerDif *= KP;

    if(Error > 8){
      RightDriveMotors.setVelocity(motorPower*powerDif + 5, pct);
      RightDrive.setVelocity(motorPower*powerDif + 5, pct);
      LeftDriveMotors.setVelocity(motorPower*powerDif + 5, pct);
      LeftDrive.setVelocity(motorPower*powerDif + 5, pct);

    } else if(Error < -8){
      RightDriveMotors.setVelocity(motorPower*powerDif - 5, pct);
      RightDrive.setVelocity(motorPower*powerDif - 5, pct);
      LeftDriveMotors.setVelocity(motorPower*powerDif - 5, pct);
      LeftDrive.setVelocity(motorPower*powerDif - 5, pct);

    }

    vex::task::sleep(20); // sleep to not overload the system
  }

  StopMotorsEnd(); //instead of stopping motors, wait(delay in seconds, seconds)?

}



void TurnInPlace(float Deg, double slowDown, int minSpeed){ 
  //Deg desired degrees
  //inert current degress

  float inertDif = Deg-inert; //degrees need to turn

  RightDriveMotors.spin(forward); // fires up motors
  RightDrive.spin(forward); 
  LeftDriveMotors.spin(forward);
  LeftDrive.spin(forward);
   
   /* in the case of when a robot turns from 1 degree to 360
      it only really turns 1 degree, but it will read as -359 degrees, 
      that's why this loop is here. */

  while (inertDif*inertDif > 0.5){ // was 0.5 
      inertDif =  Deg-inert; //constantly check
   
    if(inertDif > 180){ //can turn the other direction faster
      inertDif -=360;
    }
     if(inertDif < -180){ //can turn the other direction faster
      inertDif +=360;
    }
  
    //set speed depending on distance away 
    //fabs = float abs value
    double inertDifSquared = inertDif* fabs(inertDif); //keeps its original direction (+ or -)
    double fracOfInertDif = slowDown * fabs(inertDif); //PID quadratic part-- goes in denominator, if slowDown was 1 would be linear
    double speedNeeded = inertDifSquared/fracOfInertDif;
    if (inertDif>0){  

      RightDriveMotors.setVelocity(-speedNeeded - minSpeed ,pct);//right ones need to be reversed
      RightDrive.setVelocity(-speedNeeded - minSpeed ,pct);//right ones need to be reversed
      LeftDriveMotors.setVelocity(speedNeeded + minSpeed ,pct); //minspeed prevents not moving when really close to target
      LeftDrive.setVelocity(speedNeeded + minSpeed ,pct); //minspeed prevents not moving when really close to target

      //original long code
      // TR.setVelocity(((-inertDif)* fabs( inertDif))/(slowDown * abs((int) inertDif))-minSpeed,pct);// fun pid stuff
      // TL.setVelocity(((inertDif)* fabs( inertDif))/(slowDown * abs((int) inertDif))+minSpeed,pct); //fabs = float abs value
      // BR.setVelocity(((-inertDif)* fabs( inertDif))/(slowDown * abs((int) inertDif))-minSpeed,pct); //quadratic
      // BL.setVelocity(((inertDif)* fabs( inertDif))/(slowDown * abs((int) inertDif))+minSpeed,pct);
    }else{
      RightDriveMotors.setVelocity(-speedNeeded + minSpeed ,pct);// same idea as above, but +minSpeed bc inertDif is neg
      RightDrive.setVelocity(-speedNeeded + minSpeed ,pct);
      LeftDriveMotors.setVelocity(speedNeeded - minSpeed ,pct);
      LeftDrive.setVelocity(speedNeeded - minSpeed ,pct);

    }
      //linear version for understanding
      // TR.setVelocity(-inertDif+minSpeed,pct);
      // TL.setVelocity(inertDif-minSpeed,pct);
      // BR.setVelocity(-inertDif+minSpeed,pct);
      // BL.setVelocity(inertDif-minSpeed,pct);

    vex::task::sleep(20); // sleep to not overload the system
  }


  StopMotorsEnd();// stops all motors
}