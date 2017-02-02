# Drone
My attempt at an Arduino powered drone w/ own flight controller

###Notes:###

* Libraries must be placed in the Arduino/library folder
* All libraries are unchanged except servotimer2 was changed to avoid error with new Arduino IDE

####Jan 29th, 2017:####

* Changed everything to functions
* Confirmed that wire interferes with servo
* Servo with wire is better than servo2 with wire
* Confirmed that wire interferes with servo2….
* http://playground.arduino.cc/Main/SoftwareI2CLibrary  This can communicate with i2c four times faster, that might make a difference
* Found code that shed some light: http://www.brokking.net/ymfc-al_main.html
 * hardcode servo routine so that it only runs when the i2c isn't running
* IMU_values() takes 10ms to run...... don't know why it is so long

###Feb 1, 2017:###

* Hardcoded servo routine works perfectly even with the Wire.h library
* Receiver interrupt routine still messes with it, also not writing the pins to HIGH might be making the values not accurate, can't tell but they seem pretty messed up right now
 * Keep in mind that this changes the ESC while loop whenever I adjust the pins HIGH or LOW
* I busted off one of the battery leads so I need to solder that back on now 
	