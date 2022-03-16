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

#define MOTION_SPEED_INTERVAL 20  // milliseconds
#define MAX_ACTUATOR_INSTANCES 4

class BTS7960AD {
    public:
        //Constructor
        BTS7960AD(bool R_EN, int R_PWM, int R_IS, bool L_EN, int L_PWM, int L_IS, int sensor, float strokeLength, void(*_startCallback)(int), void(*_endCallback)(int), bool debug);

        //Methods
        void init();                                                        //reset the actuator id.
        void begin();                                                       //Initializes the actuator.
        void enable(bool value);                                            //Enable BTS7960 driver pins

        static int getInstanceCount();                                      //Get number of instances created with constructor

        void calibrationValues(int min, int max);                           //Adds calibration data to eeprom
        void calibrate(int speed = 127);                                    //calibrate min / max length
        void recalibrate(int speed = 127);                                  //recalibrate and save to eeprom

        static float averageSpeed ();                                       //returns average speed for all actuators
        void setSpeed(int newSpeed = 127);                                  //Set speed for individual constructor
        
        void stop();                                                        //stops the actuator  
        void moveToLimit(int direction, int speed);                         //General control using min max readings
        void moveToPercent(BTS7960AD* instance, int percent, int speed); //Control Actuator by percent (0-100) and direction (Retract-Extend)
        void moveTo(uint32_t newPosition);                                  //moves individual actuator
        void moveAllTo(uint32_t position);                                  //uses existing speed of all actuators to move to position in mm
        void moveAllToAvgSpeed(uint32_t position);                          //uses average speed of all actuators to move to position in mm
        void moveAllToSetSpeed(uint32_t position, int speed);               //uses defined speed of all actuators to move to position in mm

        static void update();                                               //updates millis and calls update(now)
        static void update(uint32_t now);                                   //updates position of all actuators with a new target position set

        float getPosition (void);                                           //returns position in mm
        void displayOutput();                                               //display output to serial

        //incomplete
        void calibrateAll(int speed);            

        //Variables
        int speed = 127;
        int sensorVal;
        float avg;                                                          
        
        
        
  	
    private:
        //Methods
        void setSpeedInstance(BTS7960AD* instance, int newSpeed);                               //set speed per supplied instance

        int moveToLimitCalibration(int direction, int speed);                //move to limits to set min max readings
        static void moveByAnalog(BTS7960AD* instance, int speed);
        static void setTargetPosition(BTS7960AD* instance, int position);
        
        static void driveActuator(BTS7960AD* instance, int direction, int speed);             //Control by direction (1= extend, 0= stop, -1= retract) and speed (0-255)
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
        double version = 2.00;
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

        //new
        enum MotionState{
            ACTUATOR_STANDBY,
            ACTUATOR_MOVING,
        };
        void(*startCallback)(int);
        void(*endCallback)(int);
        static BTS7960AD* instanceAddress;
        static int instanceCount;
        bool actuatorState = false;
        MotionState motionState = ACTUATOR_MOVING;
        uint32_t lastMotionMillis = 0;
        int currentPosition;
        int targetPosition;
        int newPosition;
        uint32_t sensedChangedMillis;
        

};
#endif
