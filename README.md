BTS7960 Linear Actuator Driver for potentialometer feedback linear actuator
============

# How to use

## Define object
`BTS7960AD actuator(R_EN, R_PWM, R_IS, L_EN, L_PWM, L_IS, sensor, strokeLength, debug);`

## In set up
`actuator.begin();` sets up actuator pins defined in object

`actuator.enable(true);`  enable driver pins R_EN and L_EN bringing them high

`actuator.calibrate();`   fully extends and retracts to record min max potetiometer values


## In loop

### direction
extend  = 1
stop    = 0
retract = -1

### speed
input value between 0-255

`actuator.controlActuator(1, 127);` direction and speed

`actuator.actuateByDistance(75, 1, 50);` distance mm, direction and speed


Coded by

``

     ____  _       _ __        __   ____  _      __  __

    / __ \(_)___ _(_) /_____ _/ /  / __ )(_)____/ /_/ /_

   / / / / / __ `/ / __/ __ `/ /  / __  / / ___/ __/ __ \

  / /_/ / / /_/ / / /_/ /_/ / /  / /_/ / / /  / /_/ / / /

 /_____/_/\__, /_/\__/\__,_/_/  /_____/_/_/   \__/_/ /_/
  
         /____/                                          

``
