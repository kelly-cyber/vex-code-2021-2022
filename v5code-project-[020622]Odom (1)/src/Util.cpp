#include "vex.h"
#include "Util.h"
#include "MovementFunc.h"

void Correct(){
  inertTimeError -= 0.00007;
  inert =  (InternalSensor.heading(degrees) + inertTimeError + InternalSensor.rotation() * 0.021388);
}

void Init(){

  StopMotorsEnd(); // stops all the motors
    
  InternalSensor.calibrate();
  while(InternalSensor.isCalibrating() ==true){ // calibrates inertial sensor
    vex::task::sleep(250);
  }

  // LeftEncoder.resetPosition();//resets encoder values
  // RightEncoder.resetPosition();
  // HorizontalEncoder.resetPosition();

  // yTurnOffset = 234; /* how may degrees the Y encoder rotates in one 360 deg rotation of robot*/

  // xTurnOffset = 1603; // how may degrees the X encoder rotates in one 360 deg rotation of robot
  inertError = 1; //0.02365; // error of inertial sensor being used. Enter a number like 1.01 for 1% adjustment. 


}