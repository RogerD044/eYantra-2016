/*  
 * Team Id: 0971 
 * Author List: Soumen Das, Mohit Bihany 
 * Filename: MAIN.c 
 * Theme: Transporter Bot 
 * Functions: setup(),loop() 
 * Global Variables: data,in1Pin,in2Pin,in3Pin,in4Pin,NORTH,colour,angle,flag
 */ 
#include<Stepper.h>

unsigned int data;
int in1Pin = 7;
int in2Pin = 8;
int in3Pin = 9;
int in4Pin = 10;
int NORTH = 1;
int colour = 1;           //1 = RED;  2 = BLUE;   3 = YELLOW;   4 = GREEN;
int angle;
int flag = 0;

// define the number of steps
#define STEPS 512
 
Stepper motor(STEPS, in1Pin, in2Pin, in3Pin, in4Pin); 
 
void setup()
{
  Serial.begin(9600);
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  pinMode(in3Pin, OUTPUT);
  pinMode(in4Pin, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(6, OUTPUT);
  digitalWrite(11,HIGH);
  digitalWrite(6,HIGH);
  motor.setSpeed(5);
 }
 
void loop()
{
  
  if(Serial.available()>0)
  {
    data = Serial.read();
    //flag is set to 1 only when data is received.
    flag = 1;
  }
  if( data == 11)
  colour = 1;
  
  else if( data == 12)
  colour = 2;
 
  else if( data == 13)
  colour = 3;
 
  else if( data == 14)
  colour = 4;

  //Point to the North direction respective to the previous North Direction, calculates the angle to be rotated.
  angle = (colour - NORTH) * 90;
  // Change the NORTH value to the current colour pointing the North direction
  NORTH = colour;
  // If data received, rotate according to the angle
  if(flag == 1)
  motor.step(angle/1.79);
  flag = 0;
}
