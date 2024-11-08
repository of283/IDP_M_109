#include "Global.h"
#include "Offset.h"

/*Navigation*/
int get_dir(int dir, int global) {
  switch (global) {
    case 0:
      return dir;
    case 1:
      switch (dir) {
        case 0: return 2;
        case 1: return 0;
        case 2: return 3;
        case 3: return 1;
      }
    case 2:
      switch (dir) {
        case 0: return 1;
        case 1: return 3;
        case 2: return 0;
        case 3: return 2;
      }
    case 3:
      switch (dir) {
        case 0: return 3;
        case 1: return 2;
        case 2: return 1;
        case 3: return 0;
      }
  }
}

int Navigate(int start,int destination)
{
  bool Visited[22] = {false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false};//hasnt visited any nodes yet
  Visited[start] = true;
  byte currentNode = start;
  int result[2][22] = {{10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000},{10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000,10000}};//stores shortest distance to node and previous node - use stack to get order
  result[0][start] = 0;
  result[1][start] = start;
  int distanceToCurrent = 0;
  byte NextNode = start;
  byte distanceToNext = 255;
  for (int i = 0; i < 22; i++){
    currentNode = NextNode; //update current node
    distanceToCurrent = result[0][currentNode]; //update distance to current node
    distanceToNext = 255;
    NextNode = 255; //placeholder NextNode
    for (int j = 0; j < 22 ; j++)
    {
      if (navMatrix[0][currentNode][j] > 0)
      {
        if (distanceToCurrent + 1 < result[0][j])
        {
          result[1][j] = currentNode;
          result[0][j] = 1 + distanceToCurrent; //update shortest distance to all adjacent nodes
        }
      }
    }
    for (int k = 0; k < 22; k++){
      if ((result[0][k] < distanceToNext) && (!Visited[k])){
        distanceToNext = result[0][k];
        NextNode = k;
      }
    }
    Visited[NextNode] = true;
    delay(10);
  }
  byte len = result[0][destination];
  byte order[len+1];
  order[len] = destination;
  active = destination;
  for (int i = len - 1; i >=0; i--){
    order[i] = result[1][active];
    active = order[i];
  }
  for (int i = 0; i < len; i++){
    flashblue();
    delay(10);
    active = order[i];
    byte local_dir = navMatrix[1][active][order[i+1]];
    byte dir = get_dir(local_dir, global_dir);
    byte delta = global_dir;
    if (dir == 0){
      dforwards(255);
      if (order[i+1] != 0) forwards(normal_speed);
      else{
        for (int i = 0; i < 5; i++){
          dforwards(255);
        }
      }
    }
    else if(dir == 1){
      turn_left();
      dforwards(normal_speed);
      if ((order[i+1] != 15) && (order[i+1] != 17)) forwards(normal_speed);
      else servo1.write(open);
      if (global_dir == 0) delta = 1;
      if (global_dir == 1) delta = 3;
      if (global_dir == 2) delta = 0;
      if (global_dir == 3) delta = 2;
    }
    else if(dir == 2){
      turn_right();
      dforwards(normal_speed);
      if ((order[i+1] != 15) && (order[i+1] != 17)) forwards(normal_speed);
      else servo1.write(open);
      if (global_dir == 0) delta = 2;
      if (global_dir == 1) delta = 0;
      if (global_dir == 2) delta = 3;
      if (global_dir == 3) delta = 1;
    }
    else{
      dbackwards(150);
      backwards(150);
      dbackwards(150);
    }
    global_dir = delta;
  }
  return destination;
}

/*Motor Control*/
void restore_motors()  // returns all the motors to their normal speeds and directions of rotation
{
  motor1->run(FORWARD);
  motor2->run(FORWARD);
  motor1->setSpeed(normal_speed);
  motor2->setSpeed(normal_speed);
  motor1->run(RELEASE);
  motor2->run(RELEASE);
}

void autocorrect(bool clock)  // this function actually makes the AGV autocorrect L/R. It is of much smaller amplitude than the 90 degree functions and is currently untested - 16/10/24
{
  int other_speed = normal_speed * 5 / 8;  // makes the speed of the non-dominant motor 3/4 of turning_speed
  //Serial.println(other_speed);
  byte LM = digitalRead(LMlinesensorPin);
  byte RM = digitalRead(RMlinesensorPin);
  byte LL = digitalRead(LLlinesensorPin);
  byte RR = digitalRead(RRlinesensorPin);
  int t = 0;
  while ((LM || RM) && !(LL || RR)) {  //turns to correct whenever a line sensor touches the line
    flashblue();
    LM = digitalRead(LMlinesensorPin);
    RM = digitalRead(RMlinesensorPin);
    LL = digitalRead(LLlinesensorPin);
    RR = digitalRead(RRlinesensorPin);
    if (clock) {
      motor1->setSpeed(normal_speed);
      motor1->run(FORWARD);

      motor2->setSpeed(other_speed);
      motor2->run(FORWARD);
    }

    else {
      motor1->setSpeed(other_speed);
      motor1->run(FORWARD);

      motor2->setSpeed(normal_speed);
      motor2->run(FORWARD);
    }
    t += 1;
  }
  restore_motors();  //turn off motors ready for next command
}

void line_following()  // this is the function that actually make the AGV follow the line and calls autocorrect. this is called in drive forwards
{

  int valLL = digitalRead(LLlinesensorPin);  // reads the LL input value
  int valLM = digitalRead(LMlinesensorPin);  // reads the LM input value
  int valRM = digitalRead(RMlinesensorPin);  // reads the RM input value
  int valRR = digitalRead(RRlinesensorPin);  // reads the RR input value
  //check inside sensors

  if (valLM) autocorrect(1);
  else if (valRM) autocorrect(0);
}

bool not_junction() {  //func checks if the junction detection sensors have been triggered
  bool LL = digitalRead(LLlinesensorPin);
  bool RR = digitalRead(RRlinesensorPin);
  if (LL || RR) return false;
  else return true;
}

void forwards(int speed)  // this code just moves the AGV forwards at it's normal speed
{
  while (not_junction()) {
    motor1->run(FORWARD);
    motor2->run(FORWARD);
    motor1->setSpeed(speed);
    motor2->setSpeed(speed);
    line_following();
    flashblue();
  }
  restore_motors();
}

void dforwards(byte speed) {
  motor1->run(FORWARD);
  motor2->run(FORWARD);
  motor1->setSpeed(speed);
  motor2->setSpeed(speed);
  delay(400);
  restore_motors();
}

void backwards(int speed)  // this code just moves the AGV backwards at it's normal speed
{
  while (not_junction()) {
    motor1->run(BACKWARD);
    motor2->run(BACKWARD);
    motor1->setSpeed(speed*BACK_MOTER1);
    motor2->setSpeed(speed);
    flashblue();
  }
  restore_motors();

  //dforwards(255);
  // this has been added by oscar to counteract the fact that now the junction sensors are BEHIND the motors so they reach the junction after the
  //restore_motors();
}

void dbackwards(byte speed) {
  motor1->run(BACKWARD);
  motor2->run(BACKWARD);
  motor1->setSpeed(speed*BACK_MOTER1);
  motor2->setSpeed(speed);
  delay(300);
  restore_motors();
}

void turn_left() {
  int S = 0;
  // turn clockwise
  motor1->run(FORWARD);
  motor2->run(BACKWARD);
  motor1->setSpeed(turning_speed);
  motor2->setSpeed(turning_speed);

  delay(1200);                          // spare time for the RR line sensor to leave the white line
  while (S < 1) {                      // this loop continuously checks if the furthest line sensor has found the white line again so it knows when the turn is finished
    S = digitalRead(RMlinesensorPin);  // reads the RR input value
    flashblue();
  };                                   // wait until the sensor detects white line again

  delay(TURN_LEFT_EXDELAY);  //small extra turning
  // this little cluster of code makes the robot move off of the junction so it doesn't get confused
  restore_motors();
  motor1->run(FORWARD);
  motor2->run(FORWARD);
  motor1->setSpeed(255);
  motor2->setSpeed(255);
  restore_motors();
}

void turn_right() {
  int S = 0;

  // turn anticlockwise
  motor1->run(BACKWARD);
  motor2->run(FORWARD);
  motor1->setSpeed(turning_speed);
  motor2->setSpeed(TURN_RIGHT_MOTER2);

  delay(1200);  // spare time for the LL line sensor to leave the white line

  while (S < 1) {     
    flashblue();                 // this loop continuously checks if the furthest line sensor has found the white line again so it knows when the turn is finished
    S = digitalRead(LMlinesensorPin);  // reads the RR input value
  }

  delay(TURN_RIGHT_EXDELAY);  //delay to keep turning onto line

  // this little cluster of code makes the robot move off of the junction so it doesn't get confused
  restore_motors();
  motor1->run(FORWARD);
  motor2->run(FORWARD);
  motor1->setSpeed(255);
  motor2->setSpeed(255);
  restore_motors();
}

void flashred()
{
  digitalWrite(ledred, HIGH);
  delay(2000);
  digitalWrite(ledred, LOW);
}

void flashgreen()
{
  digitalWrite(ledgreen, HIGH);
  delay(2000);
  digitalWrite(ledgreen, LOW);
}

void flashblue()
{
  t2 = millis();
  dt = t2  - t1;
  if (dt >= 250){
    if (blue == 0){
      blue = 1;
      digitalWrite(ledblue, HIGH);
    }
    else {
      blue = 0;
      digitalWrite(ledblue, LOW);
    }
    t1 = t2;
  }
}

float get_distance()
{
 sensity_t = analogRead(dsensor); // read the value from the sensor:
 return sensity_t * MAX_RANG / ADC_SOLUTION; // calculate distance
}

bool magnetic() //returns true or false from magnet sensor
{
  magval = digitalRead(magnet);
  if (magval == HIGH)
  {
    return true;
  }
  else 
  {
    return false;
  }
  
}

void grab()
{
  dist_t = get_distance();
  while (dist_t > 5) //gets in range
  {
    dforwards(200);
    if (p==3) line_following();
    dist_t = get_distance();
    flashblue();
  }
  p++;
  servo1.write(close);
  delay(1000);
  magbool = magnetic(); //checking if magnetic and responds with led flash
  if (magbool == true)
  {
    flashred(); 
  }
  if (magbool == false)
  {
    flashgreen();
  }
}

void ungrab()
{
  servo1.write(open);
  delay(1000);
}

void start_movement() // this code moves the robot forwards from the starting box onto the first line
{
  int starting_speed = 150; // sets the speed at the start of the program to be less than the normal speed

  delay(3000);  // gives time for people to move out of the way of our blisteringly fast robot

  byte S2 = digitalRead(LMlinesensorPin);
  byte S3 = digitalRead(RMlinesensorPin);
  byte S1 = digitalRead(LLlinesensorPin);
  byte S4 = digitalRead(RRlinesensorPin);

  while (!(S1 || S4)) // the robot just moves straight forward until it reaches the edge of the box (detected by the furthest left and right sensors)
  {
    dforwards(starting_speed);
    S2 = digitalRead(LMlinesensorPin);
    S3 = digitalRead(RMlinesensorPin);
    S1 = digitalRead(LLlinesensorPin);
    S4 = digitalRead(RRlinesensorPin);
  }

  //first_movement = false;

  dforwards(normal_speed);  // now we hope that the line is in the center of that edge so we move forwards for a small amount of time to clear the edge
  restore_motors(); // stops the robot from moving
}

void IDP(){
  byte active = 1;
  byte drop_off[4] = {12,5,13,8}; //all drop off locations on grid
  byte pick_up[4] = {15,15,17,17}; //all pick up locations on grid
  bool done = false; //have all packages been delivered
  bool carrying_box = false; //are we currently carrying a box
  int count = 0;
  while (!done){
    flashblue();
    if (carrying_box == true)
    {
      byte next_drop = 0;
      if (magnetic())
      {
        next_drop = 4; //drop off point for contaminated packages
      }
      else
      {
        for (int i = 0; i < 5; i++){
          if (drop_off[i] > 0){ //cycle through drop off locations until unused one is found
            next_drop = drop_off[i];
            drop_off[i] = 0;
            break;
          }
        }
      }
      Serial.println(next_drop);
      active = Navigate(active,next_drop); //drive to next drop off point
      if (next_drop == 4){
        servo1.write(almost_closed);
        for (int n = 0; n < 10; n++){
          dforwards(255);
        }
        ungrab();
        backwards(150);
        turn_right();
        turn_right();
        active = 4;
        global_dir = 0;
      }
      else{
        ungrab();
      }
      carrying_box = false;
      count ++;
    }
    else
    {
      byte next_pick = 0;
      for (int i = 0; i < 5; i++){ //cycle through pick up points to find next one
        if (pick_up[i] > 0){
          next_pick = pick_up[i];
          pick_up[i] = 0; //set that this pick up has now been used
          break;
        }
      }
      active = Navigate(active, next_pick); //drive to next pickup
      grab();
      carrying_box = true; //we have now picked up so are carrying the box
    }
    if (count == 4){ //if all have been dropped off then we are done - this will stop loop
      done = true;
    } 
  }
  Navigate(active,1);
  for (int n = 0; n < 4; n++){
    dforwards(255);
  }
}




void setup() {
  Serial.begin(19200);
  pinMode(LLlinesensorPin, INPUT);  // declare LL sensor with the right pin number
  pinMode(LMlinesensorPin, INPUT);  // declare LM sensor with the right pin number
  pinMode(RMlinesensorPin, INPUT);  // declare RM sensor with the right pin number
  pinMode(RRlinesensorPin, INPUT);  // declare RR sensor with the right pin number
  pinMode(ledgreen, OUTPUT);        // declare LEDs as output
  pinMode(ledred, OUTPUT);
  pinMode(ledblue, OUTPUT);
  pinMode(magnet, INPUT);   // declare magnet sensor as input
  pinMode(dsensor, INPUT);  //declare ultrasound as input
  pinMode(PTM, INPUT);      //declare ultrasound as input
  servo1.attach(10);        //moves servo to correct pin
  servo1.write(0);      //sets up servo to initial position

  if (!AFMS.begin())  //if cant start flag error
  {
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("***PROGRAM BEGIN***");
  int button = 0;
  servo1.write(open);
  while (!button){
    button = digitalRead(PTM);
  }
  start_movement();
  IDP();
}

void loop() {
  // put your main code here, to run repeatedly:
}
