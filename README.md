<div id="top"></div>

# BTS7960 Linear Actuator Driver 
## for potentiometer feedback linear actuator

**Designed and tested using STM32 Black Pill Built for Arduino.**

## How to use

### Define object

`BTS7960AD actuator(R_EN, R_PWM, R_IS, L_EN, L_PWM, L_IS, sensor, strokeLength, debug);`

### Pin out for BTS7960

1. RPWM Forward Level or PWM signal, **Active High**
2. LPWM Reverse Level or PWM signal, **Active High**
3. R_EN Forward Drive Enable Input, **Active High/ Low Disable**
4. L_EN Reverse Drive Enable Input, **Active High/Low Disable**
5. R_IS Forward Drive, Side current alarm output
6. L_IS Reverse Drive, Side current alarm output
7. Vcc +5V Power Supply micro-controller
8. Gnd Ground Power Supply micro-controller

## Methods in set up

`actuator.init();`      resets actuator id's in EEPROM.

`actuator.begin();`     sets up actuator pins defined in object and EEPROM memory positions.

`actuator.enable(true);`  enable driver pins R_EN and L_EN bringing them high. (May have issues - connected EN pins to the +ve rail so always on)

`actuator.calibrate(127);`   fully extends and retracts to record min max potentiometer values.

On initial start the calibrate function will test to see if there are values saved in EEPROM, if not it will fully extend and retract and record the min max values of the potentiometer and save to EEPROM, when the board is re-powered, the values can be pulled from EEPROM saving loading time.

 
`actuator.recalibrate(127);`   The actuators can be recalibrated by calling method, this is called from a button press in the example sketch.

<p align="right">(<a href="#top">back to top</a>)</p>

## Methods in loop

### Direction

extend  = 1             stop    = 0             retract = -1


### Speed

input value between 0-255

### Methods

`actuator.controlActuator(1, 127);` direction and speed

`actuator.actuateByDistance(75, 1, 50);` distance mm, direction and speed

<p align="right">(<a href="#top">back to top</a>)</p>

<!-- ROADMAP -->
## Roadmap

- [ ] Add Calibration for multiple actuators - Test
- [ ] Synchronized motor movement function
- [ ] Add position memory
    - [ ] Digital read buttons

See the [open issues](https://github.com/digitalbirth/BTS7960AD/issues) for a full list of proposed features (and known issues).

<p align="right">(<a href="#top">back to top</a>)</p>


<!-- CONTACT -->
## Contact

Dean Bateman 
- [https://www.linkedin.com/in/deanbateman/](https://www.linkedin.com/in/deanbateman/) 
- quotes@digitalbirth.com.au

Project Link: [https://github.com/digitalbirth/BTS7960AD](https://github.com/digitalbirth/BTS7960AD)

<p align="right">(<a href="#top">back to top</a>)</p>


## Coded by


        ____  _       _ __        __   ____  _      __  __
       / __ \(_)___ _(_) /_____ _/ /  / __ )(_)____/ /_/ /_    
	  / / / / / __ `/ / __/ __ `/ /  / __  / / ___/ __/ __ \   
	 / /_/ / / /_/ / / /_/ /_/ / /  / /_/ / / /  / /_/ / / /  
	/_____/_/\__, /_/\__/\__,_/_/  /_____/_/_/   \__/_/ /_/ 
            /____/    
        ------------ BRINGING YOUR IDEAS TO LIFE ------------                                      