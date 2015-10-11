# dcservo

This project uses and Arduino (or similar) to create a closed-loop position control for a DC motor to act 
as a replacement of a stepper motor and its drive electronics. In order to be compatible with stepper logic
controller accepts two inputs STEP and DIRECTION so an external trajectory controller can operate the motor
as it would do with a stepper.

For easy configuration of PID parameters, serial port communication is used when adjusting them. PID values can
be stored in EEPROM.

I want to thank Brook Drum for his support on this project. 
