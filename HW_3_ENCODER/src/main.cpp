//OPTICAL MOTOR ENCODER CODE, MOTOR CONTROLLER AND PID LIBRARIES

#include <Arduino.h>
#include <stdint.h>
#include "SCMD.h"
#include "SCMD_config.h"
#include "Wire.h"
#include <PID_v1.h>

SCMD myMotorDriver; //main motor driver object
const int PhotoPin= A0;
int sensorValue=0;
#define LEFT_MOTOR 0

//Pulse counting stuff
const int wheelSlits=10; //slits in 3d printed wheel
const int photoHigh=500; //high and low photo resistor values found by testing
const int photoLow=400;
unsigned long pulseCount=0;
const unsigned long stallTimeOut= 1500;//millisec 

//RPM read out variables
float RPM=0.0;
const unsigned long window = 1200;// window over RPM is calculated 1.2 sec
const unsigned long update = 400;// added to have is calculate & print RPM faster

//need to add delta T between slit times for rpm calc
unsigned long deltaTimeSlit = 0;

//PID STUFF
int desiredRPM = 180;
double Kp = 0.7, Ki = 0.35, Kd = 0.0; //P ID tuning variavles

//PID inputs and outputs Setpoint = desired RPM, Input = measured, output = motor drive 0-255
double Setpoint = 0, Input = 0, Output = 0;
//motor drive limit
const int DRIVE_MIN = 20;
const int DRIVE_MAX = 180;

//make PID object 
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);//direct = output++ if input<setpoint

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
  pinMode(8, INPUT_PULLUP); //motor pin default 8 i think
  delay (1000);
  Serial.begin(9600);
  delay(1000);
  Serial.println("Starting sketch.");
  
  //photoresistor
  pinMode(PhotoPin, INPUT);
  
  //motor and I2C setup mainly from motor controller library example
  myMotorDriver.settings.commInterface = I2C_MODE;
  myMotorDriver.settings.I2CAddress = 0x5D; //DEFAULT NEEDS TO BE 0x5D
  myMotorDriver.settings.chipSelectPin = 10;

  delay(1000); //Wait for the motor driver to power up
  while ( myMotorDriver.begin() != 0xA9 ) //Wait until a valid ID word is returned
  {
    Serial.println( "ID mismatch, trying again" );
    delay(500);
  }
  Serial.println( "ID matches 0xA9" );
  delay(500);

  //  Check to make sure the driver is done looking for slaves before beginning
  Serial.print("Waiting for enumeration...");
  while ( myMotorDriver.ready() == false );
  Serial.println("Done.");
  Serial.println();

  while ( myMotorDriver.busy() ); //Waits until the SCMD is available.
  myMotorDriver.inversionMode(1, 1); //invert motor 1

  while ( myMotorDriver.busy() );
  myMotorDriver.enable(); //Enables the output driver hardware


  //PID setup STUFF
  Setpoint = desiredRPM; 
  Input = 0;
  Output = 60;//abitrary start speed
  myPID.SetOutputLimits(DRIVE_MIN, DRIVE_MAX);
  myPID.SetSampleTime(update);//how often RPM and PID is calc/updated
  myPID.SetMode(AUTOMATIC);//enables the pid
  myMotorDriver.setDrive(LEFT_MOTOR, 0, (int)Output); //starts motor at inital speed before loop
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop()
{
  //tested abd found 30 is about 44rpm 90 is about 160rpm, too high of a speed and the accuracy fallls offff.
  // also its too fast to count above 90 with out timelapse

  sensorValue = analogRead(PhotoPin); //read photo sens
  //then we want to make a pulse detector to be able to figure out RPM somehow
  // need a debounce so that it only counts once per slit:
  static bool armed = true; // true arms for next slit
  static unsigned long lastSlitTime = 0;//time of last slit in microsec for EXTREME accuracy
  const unsigned long DebounceTime= 6000;// minimum time between each slit in microsec (6000 seems to work ok)
  unsigned long timeNow = micros();

  // if armed and photo sens sees bright slit and enough time has passed it will count a pulse
if (armed && (sensorValue > photoHigh) && ((timeNow - lastSlitTime) > DebounceTime)) {
    pulseCount++;
  //RPM reading is jumping all over so try to smooth it with moving avg
  if (lastSlitTime != 0) {
    unsigned long dt = timeNow - lastSlitTime;
    if (deltaTimeSlit == 0){ 
      deltaTimeSlit = dt; //start up value so not zero
    }else
    deltaTimeSlit = (deltaTimeSlit * 17 + dt*3) / 20;//85% old 15% new
  }//hopefully will help with smoother feedback
  lastSlitTime = timeNow;   // remember time of this slit
  armed = false;            // disarm; will re-arm when dark
}
// Rearm when sensor reads dark
if (sensorValue < photoLow) {
  armed = true;
}


//RPM Calc Part
// Delay for Serial, PID, and RPM calc
static unsigned long lastUpdate = 0;
unsigned long now = millis();
if ((now - lastUpdate) >= update) {
  lastUpdate = now; //400ms or so delay

  // Stall decection, if no slits/pluses seen, assume stopped
  unsigned long stallTime = micros() - lastSlitTime;
// if deltatime slit is 0 orstall time > stall time out is it stalled 
  bool stalled = (deltaTimeSlit == 0) || (stallTime > (stallTimeOut * 1000UL)); //1000*UL unit conversion
  if (!stalled) { //if not stalled calculate the RPM
    // RPM = 60e6 / (slits * deltaTime)
    RPM = (60000000.0f / (float)wheelSlits) / (float)deltaTimeSlit;// (float) "float cast" needed for math with ints with out would be whole number math
  } else {
    RPM = 0.0f;//if it is stalled display 0
  }

  //PID
  Input = RPM;
  Setpoint = desiredRPM;
  myPID.Compute();
  myMotorDriver.setDrive(LEFT_MOTOR, 0, (int)Output);

  // print
  Serial.print("  RPM=");     
  Serial.print(RPM, 1);
  Serial.print("  Set RPM=");    
  Serial.println(Setpoint);
}
}