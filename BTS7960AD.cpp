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

BTS7960AD::BTS7960AD(bool R_EN, int R_PWM, int R_IS, bool L_EN, int L_PWM, int L_IS, int sensorPin, float strokeLength, bool DEBUG){
	// -- pins defined
	en_R = R_EN;
	pwm_R = R_PWM;
	is_R = R_IS;

	en_L = L_EN;
	pwm_L = L_PWM;
	is_L = L_IS;

	sensor = sensorPin;
	stroke = strokeLength;

	debug = DEBUG;
}

/*
	SETUP
*/
void BTS7960AD::init(){
  if(this->debug){
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
		Serial.println(this->version);
    Serial.println(" ");
  }
  EEPROM.put(200, 0);
}

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
  this->configPosition = (12 * this->id)-11;
  this->minAnalogPosition = (12 * this->id)-8;
  this->maxAnalogPosition = (12 * this->id)-4;

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

/*
	ENABLE MOTORS
*/
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
    this->maxAnalogReading = moveToLimit(1, speed);
    EEPROM.put(this->maxAnalogPosition, this->maxAnalogReading);
	  this->minAnalogReading = moveToLimit(-1, speed);
    EEPROM.put(this->minAnalogPosition, this->minAnalogReading);
    this->mmTravel = (this->maxAnalogReading-this->minAnalogReading)/this->stroke;
    this->configValue = 1;
    EEPROM.put( this->configPosition, this->configValue );
    Serial.println("Calibration type: Initial");
  }

  if(this->debug){
    Serial.println("CALIBRATION READINGS");
    Serial.println("----------------------------------");
    Serial.print("maxAnalogReading = \t\t");
    Serial.println(this->maxAnalogReading);	
    Serial.print("minAnalogReading = \t\t");
    Serial.println(this->minAnalogReading);	
    Serial.print("mmTravel = \t\t\t");
    Serial.println(this->mmTravel);	
    Serial.println("----------------------------------");
    Serial.println(" ");
  }
  driveActuator(0, speed); 
  Serial.println("----------- CALIBRATION  COMPLETE -----------");
  Serial.println(" ");
}

/*
  Reset ConfigValue and Calibrate
*/
void BTS7960AD::recalibrate(int speed) {
  if(this->debug){
    Serial.println("----------- RECALIBRATION  STARTED -----------");
  }
  this->configValue = 0;
  EEPROM.put( this->configPosition, this->configValue );
  calibrate(speed);
}

/*
  MOVE TO ACTUATOR LIMIT
*/
int BTS7960AD::moveToLimit(int direction, int speed){
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
    driveActuator(direction, speed);
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
	
/*
	DRIVE ACTUATORS
*/
void BTS7960AD::driveActuator(int direction, int speed){
  if(this->debug){
    Serial.print("driveActuator (speed, dir) = \t");
    Serial.print(speed);
    Serial.print(", ");	
    Serial.println(direction);	
  }
  switch(direction){
    case 1:       //extension
      analogWrite(this->pwm_L, 0);
      analogWrite(this->pwm_R, speed);
      Serial.println("");
      Serial.println("ACTUATOR EXTENDING");
      break;
   
    case 0:       //stopping
      analogWrite(this->pwm_R, 0);
      analogWrite(this->pwm_L, 0);
      Serial.println("");
      Serial.println("ACTUATOR STOPPED");
      break;

    case -1:      //retraction
      analogWrite(this->pwm_R, 0);
      analogWrite(this->pwm_L, speed);
      Serial.println("");
      Serial.println("ACTUATOR RETRACTING");
      break;
  }
}

/*
  Control ACTUATORS
*/
void BTS7960AD::controlActuator(int direction, int speed){
  switch(direction){
    case 1:       //extension
      sensorVal = analogRead(sensor);
      while(sensorVal < this->maxAnalogReading){
        driveActuator(1, speed);
        displayOutput();  
        delay(20);
      }
      driveActuator(0, speed);
      break;
      
    case -1:      //retraction
      sensorVal = analogRead(sensor);
      while(sensorVal > this->minAnalogReading){
        driveActuator(-1, speed);
        displayOutput();  
        delay(20);
      }
      driveActuator(0, speed);
      break;
  }
}

void BTS7960AD::actuateByDistance(int mm, int direction, int speed) {
  switch(direction){
    case 1:       //extension
      sensorVal = analogRead(sensor);
      targetPos = sensorVal + (mm * mmTravel);
      if (targetPos > this->maxAnalogReading){
        targetPos = this->maxAnalogReading;
      }
      while(sensorVal < targetPos){
        driveActuator(1, speed);
        displayOutput();  
        delay(20);
      }
      driveActuator(0, speed);
      break;
      
    case -1:      //retraction
      sensorVal = analogRead(sensor);
      targetPos = sensorVal - (mm * mmTravel);
      if (targetPos < this->minAnalogReading){
        targetPos = this->minAnalogReading;
      }
      while(sensorVal > targetPos){
        driveActuator(-1, speed);
        displayOutput();  
        delay(20);
      }
      driveActuator(0, speed);
      break;
  } 
}

/*
	DRIVE ACTUATOR BY PERCENT
*/
void BTS7960AD::actuateByPercent(int percent, int speed) {
  sensorVal = analogRead(sensor);
  targetPos = percentToAnalog(percent);
  int direction;
  
  if(sensorVal < targetPos){
    direction = 1;
  }else if (sensorVal > targetPos){
    direction = -1;
  }else{
    direction = 0;
  }
  switch(direction){
    case 1:       //extension
      sensorVal = analogRead(sensor);
      if (targetPos > this->maxAnalogReading){
        targetPos = this->maxAnalogReading;
      }
      while(sensorVal < targetPos){
        driveActuator(1, speed);
        displayOutput();  
        delay(20);
      }
      driveActuator(0, speed);
      break;
    case -1:      //retraction
      sensorVal = analogRead(sensor);
      if (targetPos < this->minAnalogReading){
        targetPos = this->minAnalogReading;
      }
      while(sensorVal > targetPos){
        driveActuator(-1, speed);
        displayOutput();  
        delay(20);
      }
      driveActuator(0, speed);
      break;
    case 0:      //retraction
      driveActuator(0, speed);
      break;
  } 
}

/*
	STOP ACTUATORS
*/
void BTS7960AD::stop(){
    digitalWrite(this->en_R, LOW);
	digitalWrite(this->en_L, LOW);
	if(this->debug){
		Serial.println("Motor stopped");
	}	
}//stop();

/*
	PERCENT TO PWM 0-100 TO 0-255
*/
int BTS7960AD::percentToAnalog(int percent){
  return map(percent, 0,100,0,1023);
}//percentToPWM(10);


/*
	CALCULATE ACTUATOR POSITION
*/
float BTS7960AD::postion(float x, float inputMin, float inputMax, float outputMin, float outputMax){
 return (x-inputMin) * (outputMax - outputMin) / (inputMax - inputMin) + outputMin;
 //     512- 0           150 - 0.0                1023 - 0               0.0
 //     pos - min    x   stroke - min stroke   /   maxread - minread   + outputmin
 //     position     x   max stroke / max pot values
}

/*
	SERIAL OUTPUT
*/
void BTS7960AD::displayOutput(){
	sensorVal = analogRead(this->sensor);
	this->extensionLength = postion(sensorVal, float(this->minAnalogReading), float(this->maxAnalogReading), 0.0, this->stroke);
	Serial.print("Analog Reading: ");
	Serial.print(sensorVal);
	Serial.print("\tActuator extension length: ");
	Serial.print(this->extensionLength);
	Serial.println("mm");  
}
