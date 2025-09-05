The goal of the the class, is to provide user class that helps to control motors with single atmega 328pb dev board. 

Motors that library should be able to handle are:
 -traditional DC motor. 
 -servo motor
 -stepper motor. 


Focus is on simplicity usability and variability. 


There will be two types of functions:
a) Angle rotation - where controller will do its best to rotate the motor for a given angle (angle can be float)

Example: M_PI*4 will turn motor forward 4 revolutions. 

b) Continuous rotation with a given speed. 
 
 Example set motor speed to 50%. (input values will be from range -1,1). 


Interface:

three separate clases, that have one common 

//traditional DC motor. 
-------------------------------------------------------------------
/* function will provide some empirical power to the device, hoping it will result in required rotation to be more precise specify DCCalibrate parameters */
Turn(float angle, float speed); 

DCCalibrate. provide table of speeds, in which motor does 10 turns. for various speeds. 
 //example
   /turn no test. how many turns in test was performed
   { 
   speed 10%, time 2s
   speed 30%, time 1.5s
   speed 100% time 1s
   }

// this will run motor with given percentage speed
Run(float speed_percent);

//this will instruct motor to run at specific speed (if DCCalibrate is called, otherwise default behaviour is expected)
RunWithSpeed(float speed_rpm);

/* this will set a time that is expected from current robot motor rotation turn to new robot rotation.  */
RampUpTime(flaot time);

//servo motor
---------------------------------------------

///absolute turn to some angle (defined in radians)
Turn(float angle)

// tries to do move with offset, if posslible, otherwise nothing
TurnRelative(float angle)



//stepper motor

Turn(float angle, float speed); 

// this will run motor with given percentage speed
Run(float speed_percent);

//this will instruct motor to run at specific speed (if DCCalibrate is called, otherwise default behaviour is expected)
RunWithSpeed(float speed_rpm);


//this will instruct motor to run at specific speed (if DCCalibrate is called, otherwise default behaviour is expected)
RunWithSpeed(float speed_rpm);



