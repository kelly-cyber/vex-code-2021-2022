#include "COdometry.h"
#include <iostream>

    // constructor function
  Odometry::Odometry()
    {
      this->posX = 0;
      this->posY = 0;
      this->theta = 0;
      this->turnsL = 0;
      this->turnsR = 0;
    }
    
    double Odometry::currentTheta()
    {
      return this->theta * 180 / PI;
    }
  

    void Odometry::update()
    {
      double ld = 0;
      double rd = 0;
      double dtheta = 0;
      double dist = 0;
      double ROTl = vEncoder.position(turns);
      double ROTr = vEncoder.position(turns);
      double currX = 0;
      double currY = 0; 

      ROTl -= this->turnsL;
      ROTr -= this->turnsR;


      ld= PI*ROTl*WHDIA;

      rd = PI*ROTr*WHDIA;

      dtheta = (rd - ld)/(2*RW);

      dist = (ld + rd)/2;

      currX = dist*cos(this->theta+dtheta/2);

      currY = dist*sin(this->theta+dtheta/2);

      //printf("Rotation sensor -> %f, %f, %f, %f, %f\n", ROTl, ROTr, currX, currY, dtheta);
      Brain.Screen.setCursor(4, 1); 
      Brain.Screen.print("ROTl: ");
      Brain.Screen.setCursor(4, 15);  
      Brain.Screen.print(ROTl);
      wait(100, msec);
      
      Brain.Screen.setCursor(5, 1); 
      Brain.Screen.print("ROTr: ");
      Brain.Screen.setCursor(5, 15);  
      Brain.Screen.print(ROTr);
      wait(100, msec);

      Brain.Screen.setCursor(6, 1); 
      Brain.Screen.print("currX: ");
      Brain.Screen.setCursor(6, 15);  
      Brain.Screen.print(currX);
      wait(100, msec);
      
      Brain.Screen.setCursor(7, 1); 
      Brain.Screen.print("currY: ");
      Brain.Screen.setCursor(7, 15);  
      Brain.Screen.print(currY);
      wait(100, msec);

      Brain.Screen.setCursor(8, 1); 
      Brain.Screen.print("dtheta: ");
      Brain.Screen.setCursor(8, 15);  
      Brain.Screen.print(dtheta);
      wait(100, msec);
      
      this->posX += currX;
      this->posY += currY;
      this->theta += dtheta;
      if (this->theta > 2*PI)

        this->theta -= (2*PI);

      if (this->theta < -(2*PI))

        this->theta += (2*PI);

      this->turnsR += ROTr;
      this->turnsL += ROTl;
    };