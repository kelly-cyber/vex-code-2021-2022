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

}