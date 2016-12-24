# Arduino_Robot
Code and documentation for robot project

XBee Robot Version 1.0
This document describes the initial version of a robot design. This initial version is to validate the basic hardware and to get the motors and servos working. Additional hardware for determining position, time, acceleration, and GPS will be added later.

Hardware

Arduino Mega 2560 or suitable matching open hardware board (“Mega”).

•	The Mega has the extra serial ports that can be referenced directly in the sketches as “Seril1” for example. This avoids “soft serial” and other issues as the ports are directly available.

•	Power on board is enough to run extra boards to be connected.

Seeed XBee Aruino Shield

•	This shield allows an XBee to be powered by the Mega 2560. The shield just goes on top.

•	The Seeed shield, unlike the Sparkfun shields, has a jumper to set the pins to use for the XBee. In our case we jump the jumpers via a wire to a Serial2 pins. This avoids using Serial which is used to communicate to the Mega

•	XBee programmed to talk to another XBee and plugged into shield.

Adafruit 16 Servo controllers

•	This controller uses its own power source to power a number of servos. The version 1.0 robot is designed for no more than 16 servos numbered in hex from 0-F.  

•	The connection is done by I2C connection via the A4 and A5. 

•	A 4 AA battery is plugged into power feeds (listed below).

Sparkfun Serial Motor controller

•	This controller is able to control two motors using independent power and isolates the motor noise from the Mega.

•	A 9V battery is plugged into power feeds (listed below).

•	Serial1 is used to talk to the controller.

Sparkfun Serial LCD

•	Softserial is used; this is an output only device and this can be run simply by a simulated serial port and save on hardware serial. D3 and D2 are used to connect. 

•	Powered by Mega.

4 AA Battery holder

1 9V with JST connector added

1 10K Pot.

Connection Summary

Mega has Seeed XBee shield plugged in the normal way.

All jumpers are pulled on the Seeed XBee shield and jumper wires connect XBee to Serial2 (reversing Tx and Rx).

XBee is fully powered by Seeed shield.

One 10K Pot connected to Mega 5V, ground, and A0 to allow reading the value of the voltage in A0. Breadboard can be used to share 5V and ground.

Sparkfun Serial LCD one wire to D3. Powered by 5V and ground of Mega. D3 used to send information. D2 ignored.

Adafruit Servo Controller is an I2C device and is connected to A4 and A5. As we are using only one servo we have not added a cap to the 
controller.

Command structure

Commands start with AT. Examples are ATR1100 for full reverse on motor 1.

R, F, or S follow being R-Reverse Motor, F-Forward Motor, and S-Servo.

The next character is a number in hex 0-F (capitals only). This is the motor number (0-7 for motor 1, 8-F for motor 2) or the servo number 0-15 in hex.

A number 000-999 is the next value but valid values are usually 0-100 as that is the speed control for the motor. The servo value can be above 100% to turn it even more. 

The pot on A0 is used to allow for finer adjustments. It reduces the servo value. That if A0 is set to 100% of 5V then it will send, for example, 50% if 050 is sent.  Servos are controlled by a pulse value so the servo is set to move 100% of its movement. The adjustment is used to reduce this to the required movement.  Currently only on pot is used.


Serial_Echo_Mega.ino

Robot control code. Version 1.0 without any bells or whistles. 

Serial_Echo_Mega.ino

Test code to echo serial. Good test for motor control.


