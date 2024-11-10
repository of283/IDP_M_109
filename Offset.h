#ifndef OFFSET_H
#define OFFSET_H
//this fule was created because of issues with the speeds of the motors relative to eachother varying slightly and is used to tune so that the motors turn at the same speeds in various functions
//backwards and dbackwards
#define BACK_MOTER1 1.05 // The scale factor for motor1

//turn_left
#define TURN_LEFT_EXDELAY 2 //The extra delay

//turn_right
#define TURN_RIGHT_EXDELAY 0 //The extra delay
#define TURN_RIGHT_MOTER2 255 //speed of motor2
#endif
