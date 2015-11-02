# dcservo
by misan

This project uses and Arduino (or similar) to create a closed-loop position control for a DC motor to act 
as a replacement of a stepper motor and its drive electronics. In order to be compatible with stepper logic
controller accepts two inputs STEP and DIRECTION so an external trajectory controller can operate the motor
as it would do with a stepper.

For easy configuration of PID parameters, serial port communication is used when adjusting them. PID values can
be stored in EEPROM.

Servostrap project and Michael Ball's work sparked my curiosity to go create a simple solution for anyone to use.

I want to thank Brook Drum for his support on this project that started in here: https://www.youmagine.com/designs/dc-motor-closed-loop-control-software

# usage
<pre>
Available serial commands: (lines end with CRLF or LF) 
P123.34 sets proportional term to 123.34
I123.34 sets integral term to 123.34
D123.34 sets derivative term to 123.34
? prints out current encoder, output and setpoint values
X123 sets the target destination for the motor to 123 encoder pulses
T will start a sequence of random destinations (between 0 and 2000) every 3 seconds. T again will disable that
Q will print out the current values of P, I and D parameters
W will store current values of P, I and D parameters into EEPROM
H will print this help message again
A will toggle on/off showing regulator status every second
</pre>

#tuning tool
Written in Processing it allows you to see graphically the PID response to a step input while you can tune it by pressing capital P, I and D keys to increase values or p, i and d to lower them.
![Screenshot](http://i.imgur.com/3c8WySu.png "Tuning tool")
