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


//Register Callbacks
void startMovingActuator1(int value);
void stoppedMovingActuator1(int value);
void startMovingActuator2(int value);
void stoppedMovingActuator2(int value);

// Actuator 1
#define R_PWM_1 PB9                       // define pin 3 for R_PWM pin (output)
#define R_EN_1 PB7                        // define pin 2 for R_EN pin (input)
#define R_IS_1 PB5                        // define pin 5 for R_IS pin (output)
#define L_PWM_1 PB8                       // define pin 6 for L_PWM pin (output)
#define L_EN_1 PB6                        // define pin 7 for L_EN pin (input)
#define L_IS_1 PB4                        // define pin 8 for L_IS pin (output)
#define sensor_1 PA1                      // define pin 8 for L_IS pin (output)

// Actuator 2
#define R_PWM_2 PB4                       // define pin 3 for R_PWM pin (output)
#define R_EN_2 PB12                       // define pin 2 for R_EN pin (input)
#define R_IS_2 PB15                       // define pin 5 for R_IS pin (output)
#define L_PWM_2 PB3                       // define pin 6 for L_PWM pin (output)
#define L_EN_2 PB14                       // define pin 7 for L_EN pin (input)
#define L_IS_2 PB10                       // define pin 8 for L_IS pin (output)
#define sensor_2 PA2                      // define pin 8 for L_IS pin (output)

#define strokeLength 50                   // define stroke length in mm

#define debug 1                           //change to 0 to hide serial monitor debugging information or set to 1 to view

#define LEDPIN PC13
#define buttonPin PA0
int buttonState = 0;

BTS7960AD actuator1(R_EN_1, R_PWM_1, R_IS_1, L_EN_1, L_PWM_1, L_IS_1, sensor_1, strokeLength, startMovingActuator1, stoppedMovingActuator1, debug);
BTS7960AD actuator2(R_EN_2, R_PWM_2, R_IS_2, L_EN_2, L_PWM_2, L_IS_2, sensor_2, strokeLength, startMovingActuator2, stoppedMovingActuator2, debug);

void setup() {
    Serial.begin(115200);               // setup Serial Monitor to display information
    Serial.print("Total Actuators: ");
    Serial.println(BTS7960AD::getInstanceCount());

    pinMode(buttonPin, INPUT);          // set btn as input and pull up
    pinMode(LEDPIN, OUTPUT);

    actuator1.init();                   // reset actuator EEPROM id's
    actuator1.begin();                  // set up actuator pins
    actuator1.enable(true);             // enable driver pins
    actuator1.calibrate(127);           // set min / max actuator values

}

void loop() {

    buttonState = digitalRead(buttonPin);

    // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
    if (buttonState == LOW) {
        actuator.recalibrate(127);
    }
    
    //testSingle();
      testDual();


}// loop ends

void testDual(){
    BTS7960AD::update();
    static unsigned long lastMillis = 0;
    static bool toggle = true;
    if(millis() - lastMillis > 10000)
    {
        Serial.print("Moving to: ");
        Serial.println(toggle? 180:0);
        //actuator1.moveTo(toggle? 180:0);
        //actuator2.moveTo(!toggle? 180:0);
        actuator1.moveAllTo(toggle? 180:0);
        toggle = !toggle;
        lastMillis = millis();
    }
}


void testSingle(){
  Serial.println("Extending...");
  actuator1.moveToLimit(1, 127);
  delay(1000);

  Serial.println("Retracting...");
  actuator1.moveToLimit(-1, 127);
  delay(1000); // stop for 1 seconds 
}

//Callback functions
void startMovingActuator1(int value)
{
  Serial.print("Actuator1 Motion started: ");
  Serial.println(value);
  digitalWrite(LEDPIN, LOW);
}

void stoppedMovingActuator1(int value)
{
  Serial.print("Actuator1 Motion completed: ");
  Serial.println(value);
  digitalWrite(LEDPIN, HIGH);
}

void startMovingActuator2(int value)
{
  Serial.print("Actuator2 Motion started: ");
  Serial.println(value);
  //digitalWrite(LEDPIN, LOW);
}

void stoppedMovingActuator2(int value)
{
  Serial.print("Actuator2 Motion completed: ");
  Serial.println(value);
  //digitalWrite(LEDPIN, HIGH);
}