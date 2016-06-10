# dcservo
For original documentation, please, visit misan/dcservo.

#ATtiny85 version
I added some modifications.

#1. Determining the direction of the motor
As I was often modifying my design, I tend to plug in the motor with different polarity causing madness every time I tried to do first movement.
Now, when the code starts, it will try to move the motor and find out in which direction does it move in correlation to the encoder input.
If it finds no encoder inputs, it will consider either disconnected motor or disconnected encoder - for both cases the "error behavior" is simply to move the motor back-and-forth to signalize this problem.

#2. Determining minimum movement speed and minimal position
As soon as I determine the motor direction, I proceed forward and try to find minimum position of the current axis. What I do is I move the motor in reverse direction with a small multiplier of minimal movement speed until I reach a point, where the encoder no longer receives input - this means I've hit physical end of the path.
At that moment I will reset the position to -75 (and not to zero) - this will allow to set the "real life" zero slightly above the minimal position of the axis to avoid loud bang everytime user wants to move to 0 position. The -75 is hardcoded for now and works OK for my current usage, but feel free to change it either to 0 or any other number you find OK for your printer.

#3. Motor protection
I am monitoring the encoder input within regular time interval. If there is a situation where I am sending full power to the motors and the encoder position did not change for more then the predefined interval, I am trying to stop the motion. This will prevent situations whete the motor hits some problem (e.g. maximum axis position, huge obstacle, ...) and is unable to move, while the firmware still feeds full power - the motor would overheat and/or cause other significant damages.

#4. Removal of serial debug
Due to lack of pins on ATtiny85 and lack of storage, too, it made very little sense to keep the serial communication code and EEPROM code. Therefore I removed it and hardcoded the values for PID. You can use original code from Misan and his PID tuning tool to tune your motor and then use the final values within this sketch.

#5. Converting values to LONG
In some cases the 32767 values of encoder position and target step may not be enough. This is especially the case with geared motors and high resolution encoders. Therefore I moved the code to LONG, providing enough room for all situations.

#6. ATtiny85 specifics
You have to use the avrdude code to fix the reset pin - otherwise your driver will have a reset situation whenever there is a step input.
To restore the original status - e.g. when reprogramming is needed, I provided a link to HVSP fuse reset project done by Arduino, 1 NPN transistor and 6 resistors.
Also, note you SHOULD NOT use the attinycore as the core base for the tiny - for some reason (I did not investigate for now) it does not work and compiled code will not execute the way it should. Use this URL to get the needed tiny core into Arduino IDE: http://highlowtech.org/?p=1695
