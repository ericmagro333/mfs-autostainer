#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"
#include <PID_v1.h>

//
/// Interrupt information
/// Int 0 on pin 2 
/// Int 1 on pin 3 Not used
//
#define encoderI 2  // Interrupt 0 is use
#define encoderQ 4 // Only use one interrupt in this example
#define maxPos   3733 // left hand side
#define minPos   0    // right hand side 
#define maxSpeed 140
#define minSpeed 0
#define minGap 211    //gap at which the tunning is changed 1 inch appr.
#define Freq     300
#define maxWait  250 //12 millisec before stopping the motor
#define maxPositionError 2 //max error from actual desired position. It is approximately 0.032 of an inch
#define PIDsampleTime 5
//#define myPosition 3702
#define Delay     2000

//
//// Tuning parameters 5, 0.12, 0.1 worked better
double aggkP= 0.058; //0.0698; //Initial Proportional Gain 
double aggkI= 0.012;//.012; //0.135; //Initial Integral Gain 
double aggkD= 0.011;//0.01;//.012; //0.105; //Initial Differential Gain 

//// new Tuning parameters 
double conskP= 0.0110; //0.2; //Initial Proportional Gain 
double conskI=0.00220;/// 0.002; //0.01; //Initial Integral Gain 
double conskD= 0.0023;//0.0010; //0.00450; //Initial Differential Gain 


int outputSign;
unsigned int myPosition;
//
double Setpoint, Input, Output;

PID myPID(&Input, &Output, &Setpoint,aggkP,aggkI,aggkD, DIRECT);

volatile int ActualHeadPosition;   //current position will be modified with each interrupt
volatile int PreviousHeadPosition; //Previous position will be modified with each interrupt

int copyActualHeadPosition;        //create copy of variable while it is in use by main program

volatile byte blocked;                //semaphore
volatile uint8_t Speed = maxSpeed; //Speed for the dispenser head
volatile unsigned long lastPositionTime;
volatile int setPoint;
volatile int Gap;
volatile int lastError;
volatile int Integral;

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
//
//// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *dispenserDCMotor = AFMS.getMotor(1);
//
void setup() {
     Serial.begin(9600);           // set up Serial library at 9600 bps also this is the baud rate per second for the input?
     pinMode(encoderI, INPUT); //pin #2 is assigned as an input
     pinMode(encoderQ, INPUT); 
     attachInterrupt(0, handleEncoder, CHANGE); 
 
     AFMS.begin(Freq);  
   
    // Set the speed to start, from 0 (off) to 255 (max speed)
    dispenserDCMotor->setSpeed(Speed);
    dispenserDCMotor->run(FORWARD);
    // turn on motor
    dispenserDCMotor->run(RELEASE);
     
    Setpoint = maxPos;
    
    //setup PID frequency and limits
    myPID.SetSampleTime(PIDsampleTime);
    myPID.SetOutputLimits(minSpeed, maxSpeed);

    //turn the PID on
    myPID.SetMode(AUTOMATIC);

     //Initialize procedure should be the first thing.
     //find zero position. This procedure needs to change to look for limit sensor QRD1114.
     //Current assumption will not work if the dispenser gets stuck before reaching the end
     
    Initialize();
}

void loop() {
  delay(Delay);
  Serial.print(", ActualHeadPosition: ");Serial.println(ActualHeadPosition);
  //jus in case motor stops moving limit the voltage/current applied
  if (!isMotorMoving()) moveStop();
  --
  myPosition = 1280;
  moveTo(myPosition);
  Serial.print("Position 1280, ActualHeadPosition: ");Serial.println(ActualHeadPosition);
  delay(Delay);
  moveTo(100); //back to resting area
  delay(Delay);
  //--
  myPosition = 1670;
  moveTo(myPosition);
 Serial.print("Position 1670, ActualHeadPosition: ");Serial.println(ActualHeadPosition);
  delay(Delay);
  moveTo(100);
  delay(Delay);
  //--
  myPosition = 2050;
  moveTo(myPosition);
 Serial.print("Position 2050, ActualHeadPosition: ");Serial.println(ActualHeadPosition);
  delay(Delay);
  moveTo(100);
  delay(Delay);
  //--
  myPosition = 2440;
  moveTo(myPosition);
 Serial.print("Position 2440, ActualHeadPosition: ");Serial.println(ActualHeadPosition);
  delay(Delay);
  moveTo(100);
  delay(Delay);
  //--
  myPosition = 2830;
  moveTo(myPosition);
 Serial.print("Position 2830, ActualHeadPosition: ");Serial.println(ActualHeadPosition);
  delay(Delay);
  moveTo(100);
  delay(Delay);

}

void handleEncoder(){  
  if (!blocked){ 
    lastPositionTime = millis();
    if (copyActualHeadPosition !=0){
      PreviousHeadPosition = copyActualHeadPosition;
      copyActualHeadPosition = 0;
    }
    else {
      PreviousHeadPosition = ActualHeadPosition;
    }
    if(digitalRead(encoderI) == digitalRead(encoderQ)){
     //dispenser is moving to the left
      ActualHeadPosition++;
     }
     else {
     //dispenser is moving to the right
     ActualHeadPosition--;
     }
  }
  else { //lock is in place, update the copy of the variable
    // Do not update the last position time since we are not actually updating the real variable
    if(digitalRead(encoderI) == digitalRead(encoderQ)){
     //dispenser is moving to the left
      copyActualHeadPosition++;
     }
     else {
     //dispenser is moving to the right
     copyActualHeadPosition--;
    }
  }
}

void moveLeft(){
  dispenserDCMotor->run(FORWARD);
  dispenserDCMotor->setSpeed(Speed);  
}

void moveRight(){
  dispenserDCMotor->run(BACKWARD);
  dispenserDCMotor->setSpeed(Speed);  
}

void moveStop(){
  dispenserDCMotor->setSpeed(0);
  dispenserDCMotor->run(RELEASE);
}
boolean isMotorMoving(){
  //Set semaphore
  if (!blocked) blocked = !blocked;
  if (((millis() - lastPositionTime) > maxWait)){
    blocked = !blocked;
    return false;
  }
  else{
    blocked = !blocked;
    return true;
  }
}

void Initialize(){
    //Move to origin
    moveRight();
    while (isMotorMoving()){
  //    Serial.println("MOVING!");
  delay(10);
    }
    ActualHeadPosition = 0;
    PreviousHeadPosition = 0;
    dispenserDCMotor->setSpeed(0);
    dispenserDCMotor->run(RELEASE);
}

void moveTo(unsigned int position){
 for (;;){ // stay in the loop until new position is reached
  setPoint = position;
  
  if (!blocked){
    blocked != blocked;
    Input = ActualHeadPosition;
    Gap = setPoint - ActualHeadPosition;
    blocked != blocked;
  }
  
  outputSign = myPID.Compute();
  Speed = Output;
 
  if (outputSign != 0){
   // Serial.print("PID Output");Serial.print(outputSign * Output);  Serial.print("Speed");Serial.print(Speed);Serial.print(" Error: ");Serial.println(Gap);
  }
  else {
  //  Serial.print("PID Output");Serial.print(Output);  Serial.print("Speed");Serial.print(Speed);Serial.print(" Error: ");Serial.println(Gap);
  }
  if (Gap < minGap){
    //Getting close to setpoint so apply conservative constants
    myPID.SetTunings(conskP,conskI,conskD);
  }
  else {
    myPID.SetTunings(aggkP,aggkI,aggkD);
  }
  if (abs(Gap) < maxPositionError) {
    moveStop(); //stop moving
    break;
  }
  else if (Gap > 0){
    moveLeft(); // we are before the setpoint
  }
   else{
     moveRight(); //we are past the etpoint
  }
 }
}
