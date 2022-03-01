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

#include "Arduino.h"
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
void BTS7960AD::begin()
{
	pinMode(this->en_R, OUTPUT);
	pinMode(this->pwm_R, OUTPUT);
	pinMode(this->is_R, INPUT);	
	
	pinMode(this->en_L, OUTPUT);
	pinMode(this->pwm_L, OUTPUT);
	pinMode(this->is_L, INPUT);	

	pinMode(this->sensor, INPUT);

	if(this->debug){
		Serial.println("BTS7960 Linear Actuator Driver");
		Serial.print("version:");
		Serial.println(this->version);		
	}	
}

/*
	ENABLE MOTORS
*/
void BTS7960AD::enable(bool value) {
    digitalWrite(this->en_R, value);
	  digitalWrite(this->en_L, value);
}

void BTS7960AD::calibrate(int speed) {
	maxAnalogReading = moveToLimit(1, speed);
	minAnalogReading = moveToLimit(-1, speed);
  mmTravel = (maxAnalogReading-minAnalogReading)/stroke;
  
}

/*
  MOVE TO ACTUATOR LIMIT
*/
int BTS7960AD::moveToLimit(int direction, int speed){
  int prevReading=0;
  int currReading=0;
  do{
    prevReading = currReading;
    driveActuator(direction, speed);
    timeElapsed = 0;
    while(timeElapsed < 200){ delay(1);}           //keep moving until analog reading remains the same for 200ms
    currReading = analogRead(this->sensor);
  }while(prevReading != currReading);
  return currReading;
}
	
/*
	DRIVE ACTUATORS
*/
void BTS7960AD::driveActuator(int direction, int speed){
  switch(direction){
    case 1:       //extension
      analogWrite(this->pwm_L, 0);
      analogWrite(this->pwm_R, speed);
      break;
   
    case 0:       //stopping
      analogWrite(this->pwm_R, 0);
      analogWrite(this->pwm_L, 0);
      break;

    case -1:      //retraction
      analogWrite(this->pwm_R, 0);
      analogWrite(this->pwm_L, speed);
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
      while(sensorVal < maxAnalogReading){
        driveActuator(1, speed);
        displayOutput();  
        delay(20);
      }
      driveActuator(0, speed);
      break;
      
    case -1:      //retraction
      sensorVal = analogRead(sensor);
      while(sensorVal > minAnalogReading){
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
      if (targetPos > maxAnalogReading){
        targetPos = maxAnalogReading;
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
      if (targetPos < minAnalogReading){
        targetPos = minAnalogReading;
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
      if (targetPos > maxAnalogReading){
        targetPos = maxAnalogReading;
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
      if (targetPos < minAnalogReading){
        targetPos = minAnalogReading;
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
	extensionLength = postion(sensorVal, float(minAnalogReading), float(maxAnalogReading), 0.0, this->stroke);
	Serial.print("Analog Reading: ");
	Serial.print(sensorVal);
	Serial.print("\tActuator extension length: ");
	Serial.print(extensionLength);
	Serial.println("mm");  
}
