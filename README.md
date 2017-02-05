# Drone
My attempt at an Arduino powered drone w/ own flight controller

###Notes:###

* Libraries must be placed in the Arduino/library folder
* All libraries are unchanged except servotimer2 was changed to avoid error with new Arduino IDE

###Feb 2, 2017:###

* Found out that the data type for the loop timers was int and thus was getting reset at 32768 and giving a false statement and ending the ESC while loop too early, changed to long data type and fixed problem
* Using micros() to check the time between ESC pulses revealed that it is constant, there is slight variation in the motor
sound but the ESC pulses are constant, only thing left to check is the soldered connections since they probably suck.
* Next is balancing props/motors

###Feb 1, 2017:###

* Hardcoded servo routine works perfectly even with the Wire.h library
* Receiver interrupt routine still messes with it, also not writing the pins to HIGH in the setup makes the 
the values not accurate
 * added numbers to the ESC while loop since the receiver pins being written HIGH affects the PORTB and PORTD bytes
* I busted off one of the battery leads and soldered it back on
* Everything appears to be working, the rc values started being like 8000 so i think battery was dead on drone or receiver
try again tomorrow.  Also need to make sure it doesn't interfere with ESC writing (http://www.mappedometer.com/?maproute=583897)

####Jan 29th, 2017:####

* Changed everything to functions
* Confirmed that wire interferes with servo
* Servo with wire is better than servo2 with wire
* Confirmed that wire interferes with servo2….
* http://playground.arduino.cc/Main/SoftwareI2CLibrary  This can communicate with i2c four times faster, that might make a difference
* Found code that shed some light: http://www.brokking.net/ymfc-al_main.html
 * hardcode servo routine so that it only runs when the i2c isn't running
* IMU_values() takes 10ms to run...... don't know why it is so long

