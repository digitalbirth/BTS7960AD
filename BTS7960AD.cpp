/*
*
*     ____  _       _ __        __   ____  _      __  __  
*    / __ \(_)___ _(_) /_____ _/ /  / __ )(_)____/ /_/ /_ 
*   / / / / / __ `/ / __/ __ `/ /  / __  / / ___/ __/ __ \
*  / /_/ / / /_/ / / /_/ /_/ / /  / /_/ / / /  / /_/ / / /
* /_____/_/\__, /_/\__/\__,_/_/  /_____/_/_/   \__/_/ /_/ 
*         /____/                                          
*
* File:             BTS7960AD.cpp
* Description:      BTS7960AD is an Arduino library for controlling a linear actuator using a BTS7960 motor driver module 
* Author:           Digital Birth Pty Ltd - Bringing your ideas to life
* Contact:          digitalbirth.com.au
* Copyright:        2021
*/

/*
* --------------------------------------- 
* UPDATES
* ---------------------------------------
* Add calibration results to epprom
* Recalibration function added
* Added loading of min max potentiometer values
* 
*
*/

/*
* --------------------------------------- 
* TO DO
* ---------------------------------------
* Add Calibration for multiple actuators - Test
* Digital read btns
* Add position memory
* Sync motor option
* 
*
*/

#include "Arduino.h"
#include "EEPROM.h"
#include "BTS7960AD.h"

// 1 RPWM Forward Level or PWM signal, Active High
// 2 LPWM Reverse Level or PWM signal, Active High
// 3 R_EN Forward Drive Enable Input, Active High/ Low Disable
// 4 L_EN Reverse Drive Enable Input, Active High/Low Disable
// 5 R_IS Forward Drive, Side current alarm output
// 6 L_IS Reverse Drive, Side current alarm output
// 7 Vcc +5V Power Supply microcontroller
// 8 Gnd Ground Power Supply microcontroller

int BTS7960AD::instanceCount = 0;
BTS7960AD* instances[MAX_ACTUATOR_INSTANCES] = {nullptr};

// INITIALISATION -------------------------------------------------------------------------------------

//CONSTRUCTOR
BTS7960AD::BTS7960AD(bool R_EN, int R_PWM, int R_IS, bool L_EN, int L_PWM, int L_IS, int sensorPin, float strokeLength, void(*_startCallback)(int), void(*_endCallback)(int), bool DEBUG){
	
  instances[instanceCount++] = this;

  // -- pins defined
  en_R = R_EN;
  pwm_R = R_PWM;
  is_R = R_IS;

  en_L = L_EN;
  pwm_L = L_PWM;
  is_L = L_IS;

  sensor = sensorPin;
  stroke = strokeLength;

  //callbacks
  startCallback = _startCallback;
  endCallback = _endCallback;

  debug = DEBUG;
}

//INITIALISE EEPROM
void BTS7960AD::init(){
  if(debug){
    Serial.println("     ____  _       _ __        __   ____  _      __  __   ");
    Serial.println("    / __ \\(_)___ _(_) /_____ _/ /  / __ )(_)____/ /_/ /_  ");
    Serial.println("   / / / / / __ `/ / __/ __ `/ /  / __  / / ___/ __/ __ \\ ");
    Serial.println("  / /_/ / / /_/ / / /_/ /_/ / /  / /_/ / / /  / /_/ / / / ");
    Serial.println(" /_____/_/\\__, /_/\\__/\\__,_/_/  /_____/_/_/   \\__/_/ /_/  ");
    Serial.println("         /____/                                           ");
    Serial.println(" ");
    Serial.println("------------ BRINGING YOUR IDEAS TO LIFE ------------");
    Serial.println(" ");
		Serial.println("BTS7960 Linear Actuator Driver");
		Serial.print("version:");
		Serial.println(version);
    Serial.println(" ");
  }
  EEPROM.put(200, 0);
}

//SET UP PINS AND EEPROM
void BTS7960AD::begin()
{
	pinMode(this->en_R, OUTPUT);
	pinMode(this->pwm_R, OUTPUT);
	pinMode(this->is_R, INPUT);	
	
	pinMode(this->en_L, OUTPUT);
	pinMode(this->pwm_L, OUTPUT);
	pinMode(this->is_L, INPUT);	

	pinMode(this->sensor, INPUT);

  //set actuator id and eeprom positions
  tempID = EEPROM.get( 200, tempID );
  this->id = tempID + 1;
  EEPROM.put(200, this->id);
  this->configPosition = (15 * this->id)-14;
  this->minAnalogPosition = (15 * this->id)-10;
  this->maxAnalogPosition = (15 * this->id)-5;

  //actuatorState = true;
  //moveTo(0);
  //delay(1000);
  actuatorState = false;

	if(this->debug){
    Serial.println(" ");
    Serial.println("----------------------------------");
    Serial.print("Actuator ID: \t\t\t");
    Serial.println(this->id);	
    Serial.print("Eeprom configPosition: \t\t");
    Serial.println(this->configPosition);
    Serial.print("Eeprom minAnalogPosition: \t");
    Serial.println(this->minAnalogPosition);
    Serial.print("Eeprom maxAnalogPosition: \t");
    Serial.println(this->maxAnalogPosition);	
    Serial.println("----------------------------------");
	}	
}

//ENABLE MOTORS
void BTS7960AD::enable(bool value) {
    digitalWrite(this->en_R, value);
	  digitalWrite(this->en_L, value);
    if(this->debug){
      Serial.println(" ");
      Serial.print("Actuator ID Enabled = \t\t");
      Serial.print(this->id);	
      Serial.print(", ");
      Serial.println(value);		
    }
}

//GET INSTANCE COUNT
int BTS7960AD::getInstanceCount()
{
  return instanceCount;
}

// CALIBRATION -------------------------------------------------------------------------------------

//ADDS CALIBRATION VALUES TO EPPROM
void BTS7960AD::calibrationValues(int min, int max) {
  EEPROM.put(this->minAnalogPosition, min);
  EEPROM.put(this->maxAnalogPosition, max);
  this->configValue = 1;
  EEPROM.put( this->configPosition, this->configValue );
}

//CALIBRATE SINGLE
void BTS7960AD::calibrate(int speed) {
  if(this->debug){
    Serial.println(" ");
    Serial.println("----------------------------------");
    Serial.print("Calibration Speed = \t\t");
    Serial.println(speed);		
  }
  bool btn = false; // move to read button
  this->configValue = EEPROM.get( this->configPosition, this->configValue );
  if(this->debug){
    Serial.print("configValue: \t\t\t");
    Serial.println(this->configValue);
  }
  // config eeprom or initial config
  if (this->configValue == true){
    this->minAnalogReading = EEPROM.get( this->minAnalogPosition, this->minAnalogReading );
    this->maxAnalogReading = EEPROM.get( this->maxAnalogPosition, this->maxAnalogReading );
    this->mmTravel = (this->maxAnalogReading-this->minAnalogReading)/this->stroke;
    Serial.println("Calibration type: EEPROM");
  }else{
    this->maxAnalogReading = moveToLimitCalibration(1, speed);
    EEPROM.put(this->maxAnalogPosition, this->maxAnalogReading);
	  this->minAnalogReading = moveToLimitCalibration(-1, speed);
    EEPROM.put(this->minAnalogPosition, this->minAnalogReading);
    this->mmTravel = (this->maxAnalogReading-this->minAnalogReading)/this->stroke;
    this->configValue = 1;
    EEPROM.put( this->configPosition, this->configValue );
    Serial.println("Calibration type: Initial");
  }

  if(this->debug){
    Serial.println("CALIBRATION READINGS");
    Serial.println("----------------------------------");
    Serial.print("minAnalogReading = \t\t");
    Serial.println(this->minAnalogReading);
    Serial.print("maxAnalogReading = \t\t");
    Serial.println(this->maxAnalogReading);	
    Serial.print("mmTravel = \t\t\t");
    Serial.println(this->mmTravel);	
    Serial.println("----------------------------------");
    Serial.println(" ");
  }
  //driveActuator(0, speed); 
  Serial.println("----------- CALIBRATION  COMPLETE -----------");
  Serial.println(" ");
}

//MOVE TO ACTUATOR LIMIT
int BTS7960AD::moveToLimitCalibration(int direction, int speed){
  int prevReading=0;
  int currReading=0;
  if(this->debug){
    Serial.print("Move to limits (speed, dir) = \t");
    Serial.print(speed);
    Serial.print(", ");
    Serial.println(direction);		
  }
  do{
    prevReading = currReading;
    //driveActuator(direction, speed);
    delay(period); //keep moving until analog reading remains the same for 200ms    
    currReading = analogRead(this->sensor);
    if(this->debug){
      Serial.print("Actuator Feedback (current, previous) = \t");
      Serial.print(currReading);
      Serial.print(", ");
      Serial.println(prevReading);		
    }
  }while(prevReading != currReading);
  if(this->debug){
    Serial.println(" ");	
    Serial.println("------------- READINGS MATCHED --------------");	
    Serial.println(" ");	
  }
  return currReading;
}

//Reset ConfigValue and Calibrate
void BTS7960AD::recalibrate(int speed) {
  if(this->debug){
    Serial.println("----------- RECALIBRATION  STARTED -----------");
  }
  this->configValue = 0;
  EEPROM.put( this->configPosition, this->configValue );
  calibrate(speed);
}

// SPEED -------------------------------------------------------------------------------------

//SET SPEED
void BTS7960AD::setSpeed(int newSpeed) {
    this->speed = newSpeed;
    if(this->debug){
      Serial.println(" ");
      Serial.print("Actuator ID Speed = \t\t");
      Serial.print(this->id);	
      Serial.print(", ");
      Serial.println(this->speed);		
    }
}

//SET SPEED
void BTS7960AD::setSpeedInstance(BTS7960AD* instance, int newSpeed) {
    instance->speed = newSpeed;
    if(instance->debug){
      Serial.println(" ");
      Serial.print("Actuator instance ID Speed = \t\t");
      Serial.print(instance->id);	
      Serial.print(", ");
      Serial.println(instance->speed);		
    }
}

// MOVEMENT -------------------------------------------------------------------------------------

//STOP ACTUATORS
void BTS7960AD::stop(){
    digitalWrite(this->en_R, LOW);
	digitalWrite(this->en_L, LOW);
	if(this->debug){
		Serial.println("Motor stopped");
	}	
}//stop();

//MOVE TO LIMITS SINGLE ACTUATORS WITH MIN/MAX ANALOG VALUES
void BTS7960AD::moveToLimit(int direction, int speed){
  switch(direction){
    case 1:       //extension
      sensorVal = analogRead(sensor);
      while(sensorVal < this->maxAnalogReading){
        driveActuator(this, 1, speed);
        displayOutput();  
        delay(20);
      }
      //driveActuator(0, speed);
      break;
      
    case -1:      //retraction
      sensorVal = analogRead(sensor);
      while(sensorVal > this->minAnalogReading){
        driveActuator(this, -1, speed);
        displayOutput();  
        delay(20);
      }
      driveActuator(this, 0, speed);
      break;
  }
}

//DRIVE ACTUATOR BY PERCENT
void BTS7960AD::moveToPercent(BTS7960AD* instance, int percent, int speed) {
  instance->sensorVal = analogRead(instance->sensor);
  targetPos = percentToAnalog(percent);
  int direction;
  
  if(instance->sensorVal < targetPos){
    direction = 1;
  }else if (instance->sensorVal > targetPos){
    direction = -1;
  }else{
    direction = 0;
  }
  switch(direction){
    case 1:       //extension
      if (targetPos > this->maxAnalogReading){
        targetPos = this->maxAnalogReading;
      }
      while(instance->sensorVal < targetPos){
        driveActuator(instance, 1, speed);
        displayOutput();  
        delay(20);
      }
      driveActuator(instance, 0, speed);
      break;
    case -1:      //retraction
      if (targetPos < this->minAnalogReading){
        targetPos = this->minAnalogReading;
      }
      while(instance->sensorVal > targetPos){
        driveActuator(instance, -1, speed);
        displayOutput();  
        delay(20);
      }
      driveActuator(instance, 0, speed);
      break;
    case 0:      //retraction
      driveActuator(instance, 0, speed);
      break;
  } 
}

// public call to move single actuator into position IN mm
void BTS7960AD::moveTo(uint32_t newPosition)
{
  setTargetPosition(this, newPosition);
}

// public call to move multiple actuators into position IN mm
void BTS7960AD::moveAllTo(uint32_t position)
{
  for(int i = 0; i < instanceCount; i++)
  {
    setTargetPosition(instances[i], position);
  } 
}

// public call to move multiple actuators into position IN mm
void BTS7960AD::moveAllToAvgSpeed(uint32_t position)
{
  avg = averageSpeed();
  //Serial.print("Average speed: ");
  //Serial.println(avg);
  for(int i = 0; i < instanceCount; i++)
  {
    setTargetPosition(instances[i], position);
    setSpeedInstance(instances[i], avg);
  } 
}

// public call to move multiple actuators into position IN mm
void BTS7960AD::moveAllToSetSpeed(uint32_t position, int speed)
{
  for(int i = 0; i < instanceCount; i++)
  {
    setTargetPosition(instances[i], position);
    setSpeedInstance(instances[i], speed);
  } 
}

//MOVE BY ANALOG READING
void BTS7960AD::moveByAnalog(BTS7960AD* instance, int speed) {
  int direction;
  instance->sensorVal = analogRead(instance->sensor);
  if(instance->sensorVal > instance->newPosition){
    direction = -1; // retract
  } else if (instance->sensorVal < instance->newPosition){
    direction = 1; // extend
  }else{
    direction = 0;
  }
  switch(direction){
    case 1:       //extension
      if (instance->newPosition > instance->maxAnalogReading){
        instance->newPosition = instance->maxAnalogReading;
      }
      while(instance->newPosition < instance->sensorVal){
        driveActuator(instance, 1, speed);
        //displayOutput();  
        delay(20);
      }
      driveActuator(instance, 0, speed);
      break;
    case 0:
      driveActuator(instance, 0, speed);
      break;
    case -1:      //retraction
      if (instance->newPosition < instance->minAnalogReading){
        instance->newPosition = instance->minAnalogReading;
      }
      while(instance->newPosition > instance->sensorVal){
        driveActuator(instance, -1, speed);
        //displayOutput();  
        delay(20);
      }
      driveActuator(instance, 0, speed);
      break;
  } 
}

//DRIVE ACTUATORS without control
void BTS7960AD::driveActuator(BTS7960AD* instance, int direction, int speed){
  if(instance->debug){
    Serial.print("driveActuator (speed, dir) = \t");
    Serial.print(speed);
    Serial.print(", ");	
    Serial.println(direction);	
  }
  switch(direction){
    case 1:       //extension
      analogWrite(instance->pwm_L, 0);
      analogWrite(instance->pwm_R, speed);
      Serial.println("");
      Serial.println("ACTUATOR EXTENDING");
      break;
   
    case 0:       //stopping
      analogWrite(instance->pwm_R, 0);
      analogWrite(instance->pwm_L, 0);
      Serial.println("");
      Serial.println("ACTUATOR STOPPED");
      break;

    case -1:      //retraction
      analogWrite(instance->pwm_R, 0);
      analogWrite(instance->pwm_L, speed);
      Serial.println("");
      Serial.println("ACTUATOR RETRACTING");
      break;
  }
}

// POSITION / UPDATE -------------------------------------------------------------------------------------

//Set final position - IN mm OUT pot value
void BTS7960AD::setTargetPosition(BTS7960AD* instance, int position)  //flip-flop for the right vs. left hands.
{
  instance->targetPosition = position * instance->mmTravel;
  //error check if out of bounds
  if (instance->targetPosition < instance->minAnalogReading){
    instance->targetPosition = instance->minAnalogReading;
  }else if (instance->targetPosition > instance->maxAnalogReading){
    instance->targetPosition = instance->maxAnalogReading;
  }
}

//public call to update position if target position set
void BTS7960AD::update()
{
  update(millis());
}

// update function to check target position and update all actuators
void BTS7960AD::update(uint32_t now)
{   
  for(int i = 0; i < instanceCount; i++)
  {
    instances[i]->currentPosition = analogRead(instances[i]->sensor);
    Serial.print("Actuator: ");
    Serial.print(i);
    Serial.print(" currentPosition: ");
    Serial.println(instances[i]->currentPosition);
    switch (instances[i]->motionState)
    {
      case ACTUATOR_MOVING:
        Serial.println("ACTUATOR_MOVING");
        if(now - instances[i]->lastMotionMillis > MOTION_SPEED_INTERVAL) // time interval
        {
          if(instances[i]->targetPosition == instances[i]->currentPosition) //if target is equal to current position
          {
            if(instances[i]->actuatorState) //set to standby
            {
              instances[i]->actuatorState = false;
              instances[i]->motionState = ACTUATOR_STANDBY;
              instances[i]->endCallback(instances[i]->getPosition());
            }
          }
          else //if target is NOT equal to current position
          {
            if(!instances[i]->actuatorState) 
            {
              instances[i]->actuatorState = true;
            }
            if(instances[i]->targetPosition > instances[i]->currentPosition)  //if target is greater than current expand
            {
              instances[i]->newPosition = instances[i]->currentPosition++;
              moveByAnalog(instances[i], instances[i]->speed);
              //change to actually move actuator
              Serial.print("Actuator: ");
              Serial.print(i);
              Serial.print(" Moving to: ");
              Serial.println(instances[i]->newPosition);
            }
            else                                                            //if target is less than than current retract
            {
              instances[i]->newPosition = instances[i]->currentPosition--;
              moveByAnalog(instances[i], instances[i]->speed);
              //change to actually move actuator
              Serial.print("Actuator: ");
              Serial.print(i);
              Serial.print(" Moving to: ");
              Serial.println(instances[i]->newPosition);
            }
          }
          instances[i]->lastMotionMillis = now;
        }
        break;
      case ACTUATOR_STANDBY:
        //Serial.println("ACTUATOR_STANDBY");
        if(instances[i]->targetPosition != instances[i]->currentPosition)    
        {
          instances[i]->motionState = ACTUATOR_MOVING;
          instances[i]->startCallback(instances[i]->getPosition());
        }
      break;
    }
  }
}

// CALCULATORS / CONVERTORS -------------------------------------------------------------------------------------

//CALCULATE ACTUATOR POSITION USING POT VALUES OUTPUT mm
float BTS7960AD::postion(float x, float inputMin, float inputMax, float outputMin, float outputMax){
 return (x-inputMin) * (outputMax - outputMin) / (inputMax - inputMin) + outputMin;
 //     512- 0           150 - 0.0                1023 - 0               0.0
 //     pos - min    x   stroke - min stroke   /   maxread - minread   + outputmin
 //     position     x   max stroke / max pot values
}

// Returns current position in mm
float BTS7960AD::getPosition(void)
{
  sensorVal = analogRead(this->sensor);
  return postion(sensorVal, float(this->minAnalogReading), float(this->maxAnalogReading), 0.0, this->stroke);
}

//PERCENT TO ANALOG 0-100 TO 0-255
int BTS7960AD::percentToAnalog(int percent){
  return map(percent, 0,100,0,1023);
}//percentToAnalog(10);

//collect all speeds and calculate average
float BTS7960AD::averageSpeed()  
{ 
  long sum = 0L ;  // sum will be larger than an item, long for safety.
  int averageSpeedArray[instanceCount];
  for(int i = 0; i < instanceCount; i++){
    averageSpeedArray[i] = instances[i]->speed;
    sum += averageSpeedArray[i] ;
  } 
  //Serial.print("Average speed: ");
  //Serial.println(((float) sum) / instanceCount);

  return  ((float) sum) / instanceCount ;  // average will be fractional, so float may be appropriate.
}

//SERIAL OUTPUT
void BTS7960AD::displayOutput(){
	sensorVal = analogRead(this->sensor);
	this->extensionLength = postion(sensorVal, float(this->minAnalogReading), float(this->maxAnalogReading), 0.0, this->stroke);
	Serial.print("Analog Reading: ");
	Serial.print(sensorVal);
	Serial.print("\tActuator extension length: ");
	Serial.print(this->extensionLength);
	Serial.println("mm");  
}


// INCOMPLETE -------------------------------------------------------------------------------------

void BTS7960AD::calibrateAll(int speed) {
  // if(this->debug){
  //   Serial.println(" ");
  //   Serial.println("----------------------------------");
  //   Serial.print("Calibration Speed = \t\t");
  //   Serial.println(speed);		
  // }

  this->configValue = EEPROM.get( this->configPosition, this->configValue );
  // if(this->debug){
  //   Serial.print("configValue: \t\t\t");
  //   Serial.println(this->configValue);
  // }
  // config eeprom or initial config
  if (this->configValue == true){
    this->minAnalogReading = EEPROM.get( this->minAnalogPosition, this->minAnalogReading );
    this->maxAnalogReading = EEPROM.get( this->maxAnalogPosition, this->maxAnalogReading );
    this->mmTravel = (this->maxAnalogReading-this->minAnalogReading)/this->stroke;
    //Serial.println("Calibration type: EEPROM");
  }else{
    this->maxAnalogReading = moveToLimitCalibration(1, speed);
    EEPROM.put(this->maxAnalogPosition, this->maxAnalogReading);
	  this->minAnalogReading = moveToLimitCalibration(-1, speed);
    EEPROM.put(this->minAnalogPosition, this->minAnalogReading);
    this->mmTravel = (this->maxAnalogReading-this->minAnalogReading)/this->stroke;
    this->configValue = 1;
    EEPROM.put( this->configPosition, this->configValue );
    //Serial.println("Calibration type: Initial");
  }

  // if(this->debug){
  //   Serial.println("CALIBRATION READINGS");
  //   Serial.println("----------------------------------");
  //   Serial.print("maxAnalogReading = \t\t");
  //   Serial.println(this->maxAnalogReading);	
  //   Serial.print("minAnalogReading = \t\t");
  //   Serial.println(this->minAnalogReading);	
  //   Serial.print("mmTravel = \t\t\t");
  //   Serial.println(this->mmTravel);	
  //   Serial.println("----------------------------------");
  //   Serial.println(" ");
  // }
  //driveActuator(0, speed); 
  //Serial.println("----------- CALIBRATION  COMPLETE -----------");
  //Serial.println(" ");
}





