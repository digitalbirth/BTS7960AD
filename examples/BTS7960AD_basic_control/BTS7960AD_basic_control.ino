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

#define R_PWM PB9                       // define pin 3 for R_PWM pin (output)
#define R_EN PB7                        // define pin 2 for R_EN pin (input)
#define R_IS PB5                        // define pin 5 for R_IS pin (output)

#define L_PWM PB8                       // define pin 6 for L_PWM pin (output)
#define L_EN PB6                        // define pin 7 for L_EN pin (input)
#define L_IS PB4                        // define pin 8 for L_IS pin (output)

#define sensor PA0                      // define pin 8 for L_IS pin (output)
#define strokeLength 50                 // define stroke length in mm

#define debug 1                         //change to 0 to hide serial monitor debugging information or set to 1 to view

const int buttonPin = PA1;
int buttonState = 0;

BTS7960AD actuator(R_EN, R_PWM, R_IS, L_EN, L_PWM, L_IS, sensor, strokeLength, debug);

void setup() {
    Serial.begin(115200);               // setup Serial Monitor to display information
    actuator.init();                    // reset actuator EEPROM id's
    actuator.begin();                   // set up actuator pins
    actuator.enable(true);              // enable driver pins
    actuator.calibrate(127);            // set min / max actuator values
    pinMode(buttonPin, INPUT_PULLUP);   // set btn as input and pull up
}

void loop() {

    buttonState = digitalRead(buttonPin);

    // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
    if (buttonState == LOW) {
        actuator.recalibrate(127);
    }
    
    test();
      
}// loop ends


void test(){
  Serial.println("Extending...");
  actuator.controlActuator(1, 127);
  delay(1000);

  Serial.println("Retracting...");
  actuator.controlActuator(-1, 127);
  delay(1000); // stop for 1 seconds  

  Serial.println("Extending 50mm full speed");
  actuator.actuateByDistance(strokeLength, 1, 255);
  delay(5000); // stop for 5 seconds

  Serial.println("Retracting 25mm slowly");
  actuator.actuateByDistance(strokeLength/2, -1, 50);
  delay(5000); // stop for 5 seconds
}