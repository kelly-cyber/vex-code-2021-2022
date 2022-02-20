#pragma once
#include "vex.h"
#include "constants.h"

class Odometry 
{   

  public:

    double posX;
    double posY;
    double theta;
    double turnsL;
    double turnsR;

  

    double currentTheta();

    void update();
    
    Odometry();
};

