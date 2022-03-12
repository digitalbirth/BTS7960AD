/*
*
*     ____  _       _ __        __   ____  _      __  __  
*    / __ \(_)___ _(_) /_____ _/ /  / __ )(_)____/ /_/ /_ 
*   / / / / / __ `/ / __/ __ `/ /  / __  / / ___/ __/ __ \
*  / /_/ / / /_/ / / /_/ /_/ / /  / /_/ / / /  / /_/ / / /
* /_____/_/\__, /_/\__/\__,_/_/  /_____/_/_/   \__/_/ /_/ 
*         /____/                                          
*
* File:             BTS7960AD.h
* Description:      BTS7960AD is an Arduino library for controlling a linear actuator using a BTS7960 motor driver module 
* Author:           Digital Birth Pty Ltd - Bringing your ideas to life
* Contact:          digitalbirth.com.au
* Copyright:        2021
*/

#ifndef BTS7960AD_H
#define BTS7960AD_H

#include "Arduino.h"
#include "EEPROM.h"

class BTS7960AD {
    public:
        //Constructor
        BTS7960AD(bool R_EN, int R_PWM, int R_IS, bool L_EN, int L_PWM, int L_IS, int sensor, float strokeLength, bool debug);

        //Methods
        void init();                                              //reset the actuator id.
        void begin();                                             //Initializes the actuator.
        void enable(bool value);                                  //Enable BTS7960 driver pins
        void calibrate(int speed = 127);                          //calibrate min / max length
        void recalibrate(int speed = 127);                        //recalibrate and save to eeprom
        void controlActuator(int direction, int speed);           //General control using min max readings
        void actuateByPercent(int percent, int speed);                //Control Actuator by percent (0-100) and direction (Retract-Extend)
        void actuateByDistance(int mm, int direction, int speed); //Control Actuator by distance in mm
        void stop();                                              //stops the actuator  
        void displayOutput();

        //Variables
        int speed = 127;
        int sensorVal;
  	
    private:
        //Methods
        int moveToLimit(int direction, int speed);                //move to limits to set min max readings
        void driveActuator(int direction, int speed);             //Control by direction (1= extend, 0= stop, -1= retract) and speed (0-255)
        int percentToAnalog(int percent);
        float postion(float x, float inputMin, float inputMax, float outputMin, float outputMax);

        //Variables
        int en_R;
        int pwm_R;
        int is_R;
        int en_L;
        int pwm_L;
        int is_L;
        int sensor;
        float stroke;
        int debug;
        const unsigned long period = 200;  //the value is a number of milliseconds
        double version = 1.00;
        int id;
        int tempID = 0;
        int configPosition; 
        int configValue = 0;
        int minAnalogPosition;
        int minAnalogReading = 0;
        int maxAnalogPosition;
        int maxAnalogReading = 1023;
        int targetPos;
        float mmTravel;
        float extensionLength;
};
#endif
