/*
*     ____  _       _ __        __   ____  _      __  __  
*    / __ \(_)___ _(_) /_____ _/ /  / __ )(_)____/ /_/ /_ 
*   / / / / / __ `/ / __/ __ `/ /  / __  / / ___/ __/ __ \
*  / /_/ / / /_/ / / /_/ /_/ / /  / /_/ / / /  / /_/ / / /
* /_____/_/\__, /_/\__/\__,_/_/  /_____/_/_/   \__/_/ /_/ 
*         /____/                                          
*
* Example sketch to drive a linear actuator using a BTS7960 Driver
* 
* Written by Digital Birth Pty Ltd - Bringing your ideas to life
* visit use at digitalbirth.com.au
* 
*/

#include <BTS7960AD.h>

#define R_PWM 3                         // define pin 3 for R_PWM pin (output)
#define R_EN 4                          // define pin 2 for R_EN pin (input)
#define R_IS 5                          // define pin 5 for R_IS pin (output)

#define L_PWM 6                         // define pin 6 for L_PWM pin (output)
#define L_EN 7                          // define pin 7 for L_EN pin (input)
#define L_IS 8                          // define pin 8 for L_IS pin (output)

#define sensor 8                        // define pin 8 for L_IS pin (output)
#define strokeLength 150                // define stroke length in mm

#define debug 1                         //change to 0 to hide serial monitor debugging information or set to 1 to view


BTS7960AD actuator(R_EN, R_PWM, R_IS, L_EN, L_PWM, L_IS, sensor, strokeLength, debug);

void setup() {
    Serial.begin(115200);               // setup Serial Monitor to display information
    actuator.begin();                   // set up actuator pins
    actuator.enable(true);                  // enable driver pins
    actuator.calibrate();               // set min / max actuator values
}

void loop() {
    Serial.println("Extending...");
    actuator.controlActuator(1, 127);
    delay(1000);

    Serial.println("Retracting...");
    actuator.controlActuator(-1, 127);
    delay(1000); // stop for 1 seconds  

    Serial.println("Extending 75mm slowly");
    actuator.actuateByDistance(75, 1, 50);
    delay(5000); // stop for 5 seconds

    Serial.println("Retracting 25mm full speed");
    actuator.actuateByDistance(25, -1, 255);
    delay(5000); // stop for 5 seconds

           
}// loop ends
