#ifndef GLOBAL_H
#define GLOBAL_H

#include <Servo.h>
#include <Adafruit_MotorShield.h>
#define MAX_RANG (520)         //the max measurement value of the module is 520cm(a little bit longer than effective max range)
#define ADC_SOLUTION (1023.0)  //ADC accuracy of Arduino UNO is 10bit


Servo servo1;

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor* motor1 = AFMS.getMotor(1);//right
Adafruit_DCMotor* motor2 = AFMS.getMotor(2);//left

byte LLlinesensorPin = 6;  // Connect sensor to input pin 6
byte LMlinesensorPin = 7;  // Connect sensor to input pin 7
byte RMlinesensorPin = 8;  // Connect sensor to input pin 8
byte RRlinesensorPin = 4;  // Connect sensor to input pin 4
byte ledgreen = 12;
byte ledred = 11;
byte ledblue = 2;
byte dsensor = A1; //analog pin for distance sensor
byte magnet = 3;
byte PTM = 5; //button pin to turn on robot

int magval = 0;   // variable for reading the magnetic
int grabbed = 0;  //1 means has a package and 0 not
int noforwards = 0;
int close = 100;  //servo closed position
int almost_closed = 90; //almost closed - this allows the grabber to get over the lip into the contaminated area
int open = 0;     //servo opened position
float dist_t = 0;
float sensity_t = 0;
bool magbool = false;
int t1, t2, dt, blue;
int p = 0;

byte turning_speed = 180;  // Sets the speed for the AGV when turning
byte normal_speed = 255;   // Sets the speed for the AGV when not turning

byte navMatrix[2][22][22] = 
{//1st index = table, 2nd index = row, 3rd index = column
    {//adjacency matrix for all important points on the board - this can be varied for any configuration of board however does require several array lengths in the Navigate function in oct31.ino to be changed as well 
      {0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
      {0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
      {0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0},
      {0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0},
      {1,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0},
      {0,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
      {0,0,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
      {0,0,0,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0},
      {0,0,0,1,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0},
      {0,0,0,0,0,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0},
      {0,0,1,0,0,0,0,0,0,1,0,0,0,0,1,0,0,0,0,0,0,0},
      {0,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0},
      {0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0},
      {0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0},
      {0,0,0,0,0,0,0,0,0,0,1,0,0,1,0,0,0,0,0,0,0,1},
      {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0},
      {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,1,0},
      {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0},
      {0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,0},
      {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,1,0,1,0},
      {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,1,0,1},
      {0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0}
    },
    {//will be updated with directions to turn -> Straight ahead = 0, Left = 1, Right = 2, Reciprocate = 3, with respect to global_dir
      {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
      {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
      {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
      {0,3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
      {3,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
      {0,0,0,0,2,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
      {0,0,0,0,0,2,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
      {0,0,0,0,0,0,2,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0},
      {0,0,0,3,0,0,0,2,0,1,0,0,0,0,0,0,0,0,0,0,0,0},
      {0,0,0,0,0,0,0,0,2,0,1,0,0,0,0,0,0,0,0,0,0,0},
      {0,0,3,0,0,0,0,0,0,2,0,0,0,0,0,0,0,0,0,0,0,0},
      {0,0,0,0,3,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0},
      {0,0,0,0,0,0,0,0,0,0,0,2,0,0,0,0,0,0,0,0,0,0},
      {0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0},
      {0,0,0,0,0,0,0,0,0,0,3,0,0,2,0,0,0,0,0,0,0,0},
      {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
      {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0},
      {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2,0,0,0,0,0},
      {0,0,0,0,0,0,0,0,0,0,0,3,0,0,0,0,0,0,0,1,0,0},
      {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,3,0,0,2,0,1,0},
      {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,3,0,0,2,0,1},
      {0,0,0,0,0,0,0,0,0,0,0,0,0,0,3,0,0,0,0,0,2,0}
    }
};


byte global_dir = 0;  //0 = down graph, 1 = left, 2 = right, 3 = up - this stores the current orientation of the robot

byte active;
#endif
