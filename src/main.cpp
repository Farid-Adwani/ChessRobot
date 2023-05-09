#include <Arduino.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <math.h>

// initialize stepper motors Pins
AccelStepper StepperA(1, 22, 23);
AccelStepper StepperX(1, 2, 5);
AccelStepper StepperY(1, 3, 6);
AccelStepper StepperZ(1, 4, 7);

// initializing multistepper controllers
MultiStepper multiStepper;

// MultiStepper positions variable for 3 Steppers
long MultiPositions[3];

// Arm Angles
float PsiA = 0;
float PsiX = 0;
float PsiY = 0;
float PsiZ = 0;

//Speed Variables
int SpeedA = 800;
int SpeedX = 800;
int SpeedY = 800;
int SpeedZ = 800;

//commande Angle
String commandeString;
int commande = 0;
int commandeX = 0;
int commandeY = 0;
int commandeZ = 0;
int commandeTheta = 0;
int direction = 0;
int cmp = 0;

// initialize Limit Switch Sensor
#define FcdA A8
#define FcdX A10
#define FcdY A9
#define FcdZ A11

#define PI 3.141592653
#define JOINT1_RESOLUTION 0.060944641 /*  Joint1 resolution in degree  */
#define JOINT2_RESOLUTION 0.055333713 /*  Joint2 resolution in degree  */
#define JOINT3_RESOLUTION 0.055301333 /*  Joint3 resolution in degree  */
#define JOINT4_RESOLUTION 0.055301333 /*  Joint4 resolution in degree  */
#define X_INIT 132.86198              /*  x value of the initial position */
#define Y_INIT 0.00                   /*  y value of the initial position */
#define Z_INIT -25.3336658            /*  z value of the initial position */
#define SENS_SWITCH_DELAY 100         /*  Short delay, When switching the sens of the stepper */
#define MAX_ARM_RANGE 324             /*  The maximum range that the effector can reach */
#define l2 162.00                     /*  Length of the arm 2  */
#define l3 162.00                     /*  Length of the arm 3  */
#define l4 122.5                      /*  Length of the arm 4  */

// Convert radians to degrees
float radToDeg(double rad)
{
  return (rad * 360 / (2 * PI));
}
// Convert DEGREES to RADIANS
float degToRad(double deg)
{
  return (deg * (2 * PI) / 360);
}

// Convert Deg to Steps
int degToStep(int stepperNumber, double deg)
{
  if (stepperNumber == 1)
    return (int(deg / JOINT1_RESOLUTION));
  if (stepperNumber == 2)
    return (int(deg / JOINT2_RESOLUTION));
  if (stepperNumber == 3)
    return (int(deg / JOINT3_RESOLUTION));
  if (stepperNumber == 4)
    return (int(deg / JOINT4_RESOLUTION));
}

//Convert Steps To Deg
int StepToDeg(int stepperNumber, double step)
{
  if (stepperNumber == 1)
    return (int(step * JOINT1_RESOLUTION));
  if (stepperNumber == 2)
    return (int(step * JOINT2_RESOLUTION));
  if (stepperNumber == 3)
    return (int(step * JOINT3_RESOLUTION));
  if (stepperNumber == 4)
    return (int(step * JOINT4_RESOLUTION));
}

// Reset Position For All Steppers
void ResetPosition()
{
  StepperA.setCurrentPosition(0);
  StepperX.setCurrentPosition(0);
  StepperY.setCurrentPosition(0);
  StepperZ.setCurrentPosition(0);
}

// set speeds of steppers
void SpeedSet()
{
  StepperA.setSpeed(SpeedA);
  StepperX.setSpeed(SpeedX);
  StepperY.setSpeed(SpeedY);
  StepperZ.setSpeed(SpeedZ);
}

// First Position
void GoToFirstPosition()
{
  SpeedA = -SpeedA;
  SpeedX = -SpeedX;
  SpeedY = -SpeedY;
  SpeedZ = -SpeedZ;

  MultiPositions[0] = degToStep(1,95);
  MultiPositions[1] = degToStep(2,150);
  MultiPositions[2] = degToStep(3,30);

  StepperZ.moveTo(degToStep(4,60));
  multiStepper.moveTo(MultiPositions);

  SpeedSet();

  while (multiStepper.run() || (StepperZ.distanceToGo()!=0))
  {
    //SpeedSet(); ////for test
    multiStepper.run();//runspeedtoposition
    StepperZ.runSpeed(); //run
  }

  ResetPosition();

  SpeedX = -SpeedX;
  SpeedY = -SpeedY;

  MultiPositions[0] = int(degToStep(1, 0));
  MultiPositions[1] = int(degToStep(2, 90));
  MultiPositions[2] = int(degToStep(3, 0));

  StepperZ.moveTo(degToStep(4,-90));
  multiStepper.moveTo(MultiPositions);

  SpeedSet();

  while (multiStepper.run() || (StepperZ.distanceToGo()!=0))
  {
    //SpeedSet();   //for test
    multiStepper.run();//runspeedtoposition
    StepperZ.runSpeed(); //run
  }
}

// Initialize position
void initial_position()
{
  SpeedA = -SpeedA;
  SpeedY = -SpeedY;
  SpeedSet();

  // Run while not Reading Steppers
  while (digitalRead(FcdA))
    StepperA.runSpeed();
  while (digitalRead(FcdX))
    StepperX.runSpeed();
  while (digitalRead(FcdY))
    StepperY.runSpeed();
  while (digitalRead(FcdZ))
    StepperZ.runSpeed();

  //ResetPosition();

  GoToFirstPosition();

}

/*// Initialize Position Smooth
void initialPositionSmooth()
{

  SpeedSet();

  while (digitalRead(FcdA) || digitalRead(FcdX) || digitalRead(FcdY) || digitalRead(FcdZ))
  {
    if (!digitalRead(FcdY))
      InverseDirY();
    if (!digitalRead(FcdZ))
      InverseDirZ();

    if (digitalRead(FcdA))
      StepperA.runSpeed();
    if (digitalRead(FcdX) && digitalRead(FcdY))
      StepperX.runSpeed();
    if (digitalRead(FcdX) || digitalRead(FcdY))
      StepperY.runSpeed();
    if (digitalRead(FcdY) || digitalRead(FcdZ))
      StepperZ.runSpeed();
  }

  ResetPosition();

  InverseDirections();

  GoToFirstPosition();

 // ResetPosition();
  Serial.println("ssssssssss");
  GoToSecondPosition();

  InverseDirections();
}
*/
//  Set 2 Angles of the new target //////////////////////NOT TESTED/////////////////////
void setJointAngles(float x, float y, float z)
{

  float a = sqrt(pow(x, 2) + pow(y, 2));
  float sphereRadius1 = sqrt(a * a + pow((z + l4), 2));
  float alpha1 = (a * a + pow((z + l4), 2) + pow(l3, 2) - pow(l2, 2)) / (2 * l3);
  float beta1 = atan((z + l4) / a);

  // Verifying that the new target is in the range of the arm
  if (sphereRadius1 <= MAX_ARM_RANGE)
  {

    if (x == 0)
      PsiA = 0;
    else
      PsiA = atan(y / x); // if(xt==0) O1_t=PI/2*sign(yt); else  O1_t=atan(yt/xt);
    PsiX = acos(alpha1 / sphereRadius1) + beta1;
    PsiY = acos((a - l3 * cos(PsiX)) / l2);

    // Converting angles from radians to degrees
    PsiA = radToDeg(PsiA);
    PsiX = radToDeg(PsiX);
    PsiY = radToDeg(PsiY);
    PsiZ = -90.00;
  }
}

void setJointAnglesWithTheta(float x,float y,float z,float theta)
{
  theta = degToRad(theta);
  float a = sqrt(pow(x, 2) + pow(y, 2));
  float A = sqrt(pow(x, 2) + pow(y, 2)) - (l4*cos(theta));
  float B = z - (l4*sin(theta));
  float granda = sqrt(pow(A, 2) + pow(B, 2));
  float alpha = (a*a + pow(z, 2) + pow(l4, 2) + pow(l3, 2) - pow(l2, 2) - (2*l4*cos(theta)*a) - (2*l4*sin(theta)*z)) / (2 * l3);
  float sphereRadius = sqrt(a * a + pow((z + l4), 2));
  float beta = atan(B / A);

  // Verifying that the new target is in the range of the arm
  if (sphereRadius <= MAX_ARM_RANGE)
  {

    if (x == 0)
      PsiA = 0;
    else
    PsiA = atan(y / x); // if(xt==0) O1_t=PI/2*sign(yt); else  O1_t=atan(yt/xt);
    PsiX = acos(alpha / granda) + beta;
    PsiY = acos((granda - l3 * cos(PsiX) - l4*cos(theta)) / l2);

    // Converting angles from radians to degrees
    PsiA = radToDeg(PsiA);
    PsiX = radToDeg(PsiX);
    PsiY = radToDeg(PsiY);
    PsiZ = radToDeg(theta);
  }

}
void runToPosition(float x,float y,float z){

  setJointAngles(x,y,z);
  if(PsiA< StepToDeg(1,StepperA.currentPosition())) StepperA.setSpeed(800);
  else StepperA.setSpeed(-800);

  if(PsiX< StepToDeg(2,StepperX.currentPosition())) StepperX.setSpeed(800);
  else StepperX.setSpeed(-800);
  
  if(PsiY< StepToDeg(3,StepperY.currentPosition())) StepperY.setSpeed(800);
  else StepperY.setSpeed(-800);


  MultiPositions[0] = degToStep(1,PsiA);
  MultiPositions[1] = degToStep(2,PsiX);
  MultiPositions[2] = degToStep(3,PsiY);

  StepperZ.moveTo(degToStep(4,-90));
  multiStepper.moveTo(MultiPositions);

  SpeedSet();

  while (multiStepper.run() || (StepperZ.distanceToGo()!=0))
  {
    //SpeedSet(); ////for test
    multiStepper.run();//runspeedtoposition
    StepperZ.runSpeed(); //run
  }

/*
  PsiA = abs(PsiA - StepperA.currentPosition());
  PsiX = abs(PsiX - StepperX.currentPosition());
  PsiY = abs(PsiY - StepperY.currentPosition());*/

}

void runToPositionWithTheta(float x,float y,float z,float theta){

  setJointAnglesWithTheta(x,y,z,theta);
  if(PsiA< StepToDeg(1,StepperA.currentPosition())) StepperA.setSpeed(800);
  else StepperA.setSpeed(-800);

  if(PsiX< StepToDeg(2,StepperX.currentPosition())) StepperX.setSpeed(800);
  else StepperX.setSpeed(-800);
  
  if(PsiY< StepToDeg(3,StepperY.currentPosition())) StepperY.setSpeed(800);
  else StepperY.setSpeed(-800);

  if(PsiZ< StepToDeg(4,StepperZ.currentPosition())) StepperZ.setSpeed(800);
  else StepperZ.setSpeed(-800);


  MultiPositions[0] = degToStep(1,PsiA);
  MultiPositions[1] = degToStep(2,PsiX);
  MultiPositions[2] = degToStep(3,PsiY);

  StepperZ.moveTo(degToStep(4,PsiZ));
  multiStepper.moveTo(MultiPositions);

  SpeedSet();

  while (multiStepper.run() || (StepperZ.distanceToGo()!=0))
  {
    //SpeedSet(); ////for test
    multiStepper.run();//runspeedtoposition
    StepperZ.runSpeed(); //run
  }

/*
  PsiA = abs(PsiA - StepperA.currentPosition());
  PsiX = abs(PsiX - StepperX.currentPosition());
  PsiY = abs(PsiY - StepperY.currentPosition());*/

}

void setup()
{
  //Set Max Speed For Each Stepper Motor
  StepperA.setMaxSpeed(1000);
  StepperX.setMaxSpeed(1000);
  StepperY.setMaxSpeed(1000);
  StepperZ.setMaxSpeed(1000);

  //Adding Steppers A,X and Y to multiStepper 
  multiStepper.addStepper(StepperA);
  multiStepper.addStepper(StepperX);
  multiStepper.addStepper(StepperY);

  //Setting PinMode to each Limit Switch Sensor
  pinMode(FcdA, INPUT_PULLUP);
  pinMode(FcdX, INPUT_PULLUP);
  pinMode(FcdY, INPUT_PULLUP);
  pinMode(FcdZ, INPUT_PULLUP);

  initial_position();

  Serial.begin(9600);
  Serial.setTimeout(1000);
}

void loop()
{
  while(cmp < 4){
  if(Serial.available() > 0){
  if(cmp == 0){
    Serial.println("Donner x : ");
    commandeString = Serial.readString();
    commandeX = commandeString.toInt();
    Serial.println(commandeX);
    cmp ++;
  }
  if(cmp == 1){
    Serial.println("Donner y : ");
    commandeString = Serial.readString();
    commandeY = commandeString.toInt();
    Serial.println(commandeY);
    cmp ++; 
  }
  if(cmp == 2){
    Serial.println("Donner z : ");
    commandeString = Serial.readString();
    commandeZ = commandeString.toInt();
    Serial.println(commandeZ);
    cmp ++; 
  }
  if(cmp == 3){
    Serial.println("Donner theta : ");
    commandeString = Serial.readString();
    commandeTheta = commandeString.toInt();
    Serial.println(commandeTheta);
    cmp ++;
  }
  }
  }
  runToPosition(commandeX,commandeY,commandeZ);
  /*
  if(Serial.available() > 0){
    commandeString = Serial.readString();
    commande = commandeString.toInt();
    Serial.println(commande);
  }
  direction = 1;
  if(commande < StepToDeg(4,StepperZ.currentPosition())) direction = -1;
  
  StepperZ.moveTo(degToStep(4,90));
  StepperZ.setSpeed(800);
  while (StepperZ.distanceToGo() != 0)
  {
    StepperZ.runSpeed();
  }
 */ 
}