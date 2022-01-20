#include "vex.h"
#include "Util.h"
#include "MovementFunc.h"

// inert =  InternalSensor.heading(degrees) + InternalSensor.rotation(degrees) * inertError;// corrects the heading
//   if (inert<0){
//     inert+=360;
//   }
// }

// void Init(){

//   StopMotorsEnd(); // stops all the motors
    
//   InternalSensor.calibrate();
//   while(InternalSensor.isCalibrating() ==true){ // calibrates inertial sensor
//     vex::task::sleep(250);
//   }

//   LeftEncoder.resetPosition();//resets encoder values
//   RightEncoder.resetPosition();
//   HorizontalEncoder.resetPosition();

//   yTurnOffset = 234; /* how may degrees the Y encoder rotates 
//   in one 360 deg rotation of robot*/

//   xTurnOffset = 1603; // how may degrees the X encoder rotates in one 360 deg rotation of robot
//   inertError = 0.02365; // error of inertial sensor being used. Enter a number like 1.01 for 1% adjustment. 


// }