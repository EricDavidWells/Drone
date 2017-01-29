# Drone
My attempt at an Arduino powered drone w/ own flight controller

Notes:

* Libraries must be placed in the Arduino/library folder
* All libraries are unchanged except servotimer2 was changed to avoid error with new Arduino IDE

Jan 29th, 2017:

Markup :	* Changed everything to functions
	* Confirmed that wire interferes with servo
	* Servo with wire is better than servo2 with wire
	* Confirmed that wire interferes with servo2….
	* http://playground.arduino.cc/Main/SoftwareI2CLibrary  This can communicate with i2c four times faster, that might make a difference
	* found code that shed some light: http://www.brokking.net/ymfc-al_main.html
	** hardcode servo routine so that it only runs when the i2c isn't running
	