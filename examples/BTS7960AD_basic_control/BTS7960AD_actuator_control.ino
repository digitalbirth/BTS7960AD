/*
*     ____  _       _ __        __   ____  _      __  __  
*    / __ \(_)___ _(_) /_____ _/ /  / __ )(_)____/ /_/ /_ 
*   / / / / / __ `/ / __/ __ `/ /  / __  / / ___/ __/ __ \
*  / /_/ / / /_/ / / /_/ /_/ / /  / /_/ / / /  / /_/ / / /
* /_____/_/\__, /_/\__/\__,_/_/  /_____/_/_/   \__/_/ /_/ 
*         /____/                                          
*
* Example sketch to drive single and multiple linear actuators using a BTS7960 Driver (1 driver per actuator)
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
  #define R_IS_1 PB3                        // define pin 5 for R_IS pin (output)
#define L_PWM_1 PB8                       // define pin 6 for L_PWM pin (output)
  #define L_EN_1 PB6                        // define pin 7 for L_EN pin (input)
  #define L_IS_1 PB4                        // define pin 8 for L_IS pin (output)
#define sensor_1 PA1                      // define pin 8 for L_IS pin (output)

// Actuator 2
#define R_PWM_2 PB4                       // define pin 3 for R_PWM pin (output)
  #define R_EN_2 PB12                       // define pin 2 for R_EN pin (input)
  #define R_IS_2 PB15                       // define pin 5 for R_IS pin (output)
#define L_PWM_2 PB5                       // define pin 6 for L_PWM pin (output)
  #define L_EN_2 PB14                       // define pin 7 for L_EN pin (input)
  #define L_IS_2 PB10                       // define pin 8 for L_IS pin (output)
#define sensor_2 PA2                      // define pin 8 for L_IS pin (output)

#define strokeLength 150                   // define stroke length in mm

#define debug 0                           //change to 0 to hide serial monitor debugging information or set to 1 to view

#define LEDPIN PC13
#define buttonPin PA0
int buttonState = 0;

BTS7960AD actuator1(R_EN_1, R_PWM_1, R_IS_1, L_EN_1, L_PWM_1, L_IS_1, sensor_1, strokeLength, startMovingActuator1, stoppedMovingActuator1, debug);
BTS7960AD actuator2(R_EN_2, R_PWM_2, R_IS_2, L_EN_2, L_PWM_2, L_IS_2, sensor_2, strokeLength, startMovingActuator2, stoppedMovingActuator2, debug);

void setup() {
    Serial.begin(115200);                                   // Setup Serial Monitor to display information
    delay(3000);                                            // Delay to aid time to open serial monitor, not required
    pinMode(buttonPin, INPUT_PULLUP);                       // set btn as input and pull up
    pinMode(LEDPIN, OUTPUT);                                
    
    actuator1.init();                                       // reset actuator EEPROM id's

    Serial.print("Total Actuators: ");
    Serial.println(BTS7960AD::getInstanceCount());          // Displays number of actuators
    
    actuator1.begin();                                      // set up actuator pins
    actuator2.begin();                                      
    
    actuator1.enable(true);                                 // enable driver pins
    actuator2.enable(true);                                 

    //actuator1.calibrationValues(50, 950);                 //used to preset known config or testing
    //actuator2.calibrationValues(50, 950);                 

    actuator1.setSpeed(100);                                // Can set actuator speeds at anytime
    actuator2.setSpeed(255);

    //actuator1.recalibrate(127);                           //can recalibrate at setup to overwrite eeprom values
    //actuator2.recalibrate(127);
    
    actuator1.calibrate(127);                               // set min / max actuator values
    actuator2.calibrate(127);                               // set min / max actuator values 

    //float avg = BTS7960AD::averageSpeed();
    //BTS7960AD::averageSpeed();
}

void loop() {
    BTS7960AD::update();                                    // IMPORTANT, updates actuator positions
    
    buttonState = digitalRead(buttonPin);
    if (buttonState == LOW) {                               // Check if the pushbutton is pressed. If it is, the buttonState is HIGH:
        actuator1.recalibrate(127);
        actuator2.recalibrate(127);
    }

    //Functions work by setting a TARGET POSITION and SPEED 
    //by running multiple functions you will overright the previous
    //so select a single function to test at one time
    
    //testSingle_Toggle();                                  // Single actuator control 
    //testSingle_Limits();

    
    
    test_moveAllTo_Toggle();                               // Multiple actuator control
    //test_moveAllTo();
    //test_moveToPercent();
    //test_setSpeed_random();
    //test_moveAllToAvgSpeed();
    //test_moveAllToSetSpeed();

}// loop ends


//TEST FUNCTIONS

void test_moveToPercent(){
  actuator1.setSpeed(255);
  actuator2.setSpeed(255);
  static unsigned long lastMillis = 0;
  static bool toggle = true;
  if(millis() - lastMillis > 15000){
    Serial.println("toggle 25-75%");
    actuator1.moveToPercent(toggle? 25:75);                   //percent of stroke and speed
    actuator2.moveToPercent(toggle? 75:25);
    toggle = !toggle;
    lastMillis = millis();
  } 
}
void test_moveAllTo_Toggle(){ 
  static unsigned long lastMillis = 0;
  static bool toggle = true;
  if(millis() - lastMillis > 15000){
    Serial.print("Moving to: ");
    Serial.println(toggle? 25:75);
    actuator1.moveAllTo(toggle? 25:75);
    toggle = !toggle;
    lastMillis = millis();
  } 
}

void test_moveAllTo(){ 
  Serial.println("moveTo 55");
  actuator1.moveAllTo(55); 
}

void test_setSpeed_random(){ 
  Serial.println("Set random speed");
  actuator1.setSpeed(random(50,200));
  actuator2.setSpeed(random(50,200));
}

void test_moveAllToAvgSpeed(){  
  static unsigned long lastMillis = 0;
  static bool toggle = true;
  if(millis() - lastMillis > 10000){
    Serial.println("moveAvg 35 or 75");
    actuator1.moveAllToAvgSpeed(toggle? 35:75);
    toggle = !toggle;
    lastMillis = millis();
  } 
}

void test_moveAllToSetSpeed(){ 
  Serial.println("moveAllSetSpeed");
  actuator1.moveAllToSetSpeed(90, 75);
}

void testSingle_Toggle(){
  static unsigned long lastMillis = 0;
  static bool toggle = true;
  if(millis() - lastMillis > 20000){
    Serial.print("Moving Actuator 1 to: ");
    Serial.println(toggle? 50:20);
    actuator1.moveTo(toggle? 50:20);
    Serial.print("Moving Actuator 2 to: ");
    Serial.println(toggle? 0:30);
    actuator2.moveTo(!toggle? 0:30);
    toggle = !toggle;
    lastMillis = millis();
  }
}

void testSingle_Limits(){
    
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
  Serial.print(value);
  Serial.println("mm");
  digitalWrite(LEDPIN, LOW);
}

void stoppedMovingActuator1(int value)
{
  Serial.print("Actuator1 Motion completed: ");
  Serial.print(value);
  Serial.println("mm");
  digitalWrite(LEDPIN, HIGH);
}

void startMovingActuator2(int value)
{
  Serial.print("Actuator2 Motion started: ");
  Serial.print(value);
  Serial.println("mm");
  //digitalWrite(LEDPIN, LOW);
}

void stoppedMovingActuator2(int value)
{
  Serial.print("Actuator2 Motion completed: ");
  Serial.print(value);
  Serial.println("mm");
  //digitalWrite(LEDPIN, HIGH);
}