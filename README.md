# Overview

Idea of the project was to built a quick platform to play with an Arduino Uno.
I chose a 4WD car because this was funny and I didn't have to manage the stability of the robot.

I initially worked on the autonomous part in order to build all the libraries I would need to control a 4WD car.
Then, I added a way to remote control it and a way to switch from one mode to another to make easier the tests.

Next step will be to add the control by Voice through [Nuance technology](https://developer.nuance.com/public/index.php?task=mix)
and to add gesture control through [Myo technology](https://www.myo.com/).
This would require quite a more powerful platform and I plan to do it on my [Raspberry Pi 3 model B](https://www.raspberrypi.org/products/raspberry-pi-3-model-b/)
in another project


# Hardware

To be able to use that code as this, you will need some hardware.
This is an exhaustive list of what I used:

* 4WD Mobile platform: https://www.amazon.ca/dp/B00H8H4PSQ/

Any platform may fit. I used a quite cheap platform I was able to order from Canada.
This does the job even if I have some issue with one the motor/wheel.

* Arduino Uno: https://www.arduino.cc/en/Main/ArduinoBoardUno

That's the brain of the project.
It is used to get info from the environment and control all the other components: sonar, motors, servo, LCD & bluetooth controller.

* Sonar: http://www.robotshop.com/ca/en/hc-sr04-ultrasonic-range-finder.html

Very cheap sonar working very well when you compute distance perpendicularly to a wall.
Doesn't so well with very acute angle => Distance detection still to be improved.

* Servo Motor: https://www.adafruit.com/product/1404

I used a servo motor with feedback with is useless in that project but I wanted to try ot out.
You can use nay servo motor heavy enough to support the weight of the sonar.

I use a servo motor when I look for another direction to go to.
Idea is turn the servo motor from 10 to 170 degrees and compute the distance for every position.
Then, the car goes to the direction which seemed the more far.

* Multi-Purpose Sensor Bracket MPSH-01: http://www.robotshop.com/ca/en/lynxmotion-multi-purpose-sensor-housing.html

You will need several mechanical pieces to fix the different components to the car platform.
But I wanted to specifically put the reference to the bracket I used to fix the servo motor to the sonar since it works very well.

* Motors Shield: https://www.adafruit.com/product/1438

To avoid having way too many wiring and issue with back voltage, I chose to use a Motor Shield from Adafruit.
:warning: Be careful to solder it with **Stacking Headers** to make much more easier the wiring with the other components.

* Lithium Ion Battery: https://www.adafruit.com/product/2011

An autonomous car is useless if you have to power it with an USB cable.
To solve this issue, I chose to use an Ion Battery to power the logic supply.

* PowerBoost shield: https://www.adafruit.com/product/2078

To use in combination with the Ion Battery. This give me a way to both easity plug the battery in my project and to recharge it.

* LCD: https://www.adafruit.com/products/181

The LCD and the LCD Backpack are really optional. I added them to the project to try such component out.
I display kind of random phrases when I detect an obstacle (that's funny for my kids).

* LCD Backpack: https://www.adafruit.com/product/292

Aim of that component is to reduce number of pins used by teh LCD. That's about it.

* Bluetooth Controller: https://www.adafruit.com/product/2479

This creates a bluetooth connection with the 4WD car and allows you to control it remotely.
The car won't start if the bluetooth controller is not connected to an application.


# Software

Most of Adafruit components come with an Arduino library. These libraries have to be installed properly before using our 4WD car.

* Motors Shield: https://learn.adafruit.com/adafruit-motor-shield-v2-for-arduino/install-software

As usual, install this library by copying the folder under the `libraries` sub-directory of your Arduino installation.

* LCD Backpack: https://learn.adafruit.com/i2c-spi-lcd-backpack/connect-to-i2c#install-adafruit-liquidcrystal

As usual, install this library by copying the folder under the `libraries` sub-directory of your Arduino installation.

* Bluetooth Controller: https://learn.adafruit.com/introducing-the-adafruit-bluefruit-le-uart-friend/software

As usual, install this library by copying the folder under the `libraries` sub-directory of your Arduino installation.


:warning: In order to use objects abstracting the car controller, we will need to copy over the files available [here](https://github.com/cesquerr/Autonomous_Car/tree/master/FourWheelDriveCar) to a sub-folder under the `libraries` sub-directory of your Arduino installation.
