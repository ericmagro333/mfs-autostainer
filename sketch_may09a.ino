#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

// Interrupt information
// 0 on pin 2
// 1 on pin 3

#define encoderI 2
#define encoderQ 4 // Only use one interrupt in this example

volatile int CP; //current position will be modified with each interrupt
         
// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *dispenserDCMotor = AFMS.getMotor(1);
// You can also make another motor on port M2
//Adafruit_DCMotor *myOtherMotor = AFMS.getMotor(2);

  void setup() 
  {
    Serial.begin(9600);           // set up Serial library at 9600 bps also this is the baud rate per second for the input?
    
    pinMode(encoderI, INPUT); //pin #2 is assigned as an input
    pinMode(encoderQ, INPUT); 
    attachInterrupt(0, handleEncoder, CHANGE); 
    
    AFMS.begin();  // create with the default frequency 1.6KHz
    //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
    
    // Set the speed to start, from 0 (off) to 255 (max speed)
    dispenserDCMotor->setSpeed(150);
    dispenserDCMotor->run(FORWARD);
    // turn on motor
    dispenserDCMotor->run(RELEASE);
  }
  
  void loop()
  {
    Serial.print("Move dispenser to origin");
    dispenserDCMotor-> run(BACKWARD);
    int CP = 3732; //when should I change this value?
    int PP = 0;
    boolean a = moveTo(0); 
    
    Serial.print("Move dispenser to first slide");
    dispenserDCMotor -> run(FORWARD);   
    
  }
  
  boolean moveTo(int SP)
  {
	while(CP != SP)
	{
          if(CP == PP)
	  {
	    dispenserDCMotor-> run(RELEASE);
		if(SP != 0 )
		{
		    Serial.println("Error: Mechanism for moving dispensers has stopped.")
		}
	  }
		PP = CP; //update PP variable
		delay(100); //delay 100 ms so that the motor has time to move
	}

	dispenserDCMotor -> run(RELEASE);

    return true;
  }

  void handleEncoder()
  {
    if(digitalRead(encoderI) == digitalRead(encoderQ))
    { 
      CP++;
    }
    else
    { 
      CP--;
    }
    
   }
