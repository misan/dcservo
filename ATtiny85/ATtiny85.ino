/*
 * Final setting:
 * 0 - motor PWM1 / RX during setup
 * 1 - motor PWM2 / TX during setup
 * 2 - step input
 * 3 - encoder A
 * 4 - encoder B
 * 5 - direction
 * 
 *  To fix fuses after final program is uploaded:
 * avrdude -v -v -v -v -pattiny85 -cusbasp -Pusb -U lfuse:w:0xc1:m -U hfuse:w:0x5c:m -U efuse:w:0xff:m
 *
 * For Tiny Debug Serial use boards definition from
 * https://storage.googleapis.com/google-code-archive-downloads/v2/code.google.com/arduino-tiny/arduino-tiny-0150-0020.zip
 *
 * Should you need to get back to programming (like adjust motor parameters),
 * use HVSP programming - I used Arduino Nano with sketch from here:
 * https://arduinodiy.wordpress.com/2015/05/16/high-voltage-programmingunbricking-for-attiny/
*/
#include "avr/interrupt.h"
#include <PID_v1.h>

volatile long encoder0Pos = 0;  // real position from encoder
volatile long target1 = 0;      // destination location at any moment
volatile int moveCoef = 1;
static byte newPort;

#define M1            0  // only PWM capable pins on Tiny is 0 and 1
#define M2            1

// please adjust these 3 values for your motor!!!
// Y axis config: 
// const double kp = 12, ki = 8, kd = 0.02;
// X axis config:
// const double kp = 12, ki = 4, kd = 0.02;
// Z axis config
const double kp = 12, ki = 8, kd = 0.02;
#undef stepKoef
#define stepKoef
//#undef minSearch
#define minSearch

double input = 0, output = 0, setpoint = 0;
PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);

boolean autoReport = true;
unsigned long previousMillis = 0; // will store last time LED was updated
long lastEncPos = 0;              // will store the encoded value when last max output was measured
unsigned long curTime = 0UL;      // will store current time to avoid multiple millis() calls
unsigned long lastSafeCheck = 0;  // will store last value when 255 output was measured
unsigned long motorSafe = 4000UL; // will store the interval to protect the motor - 4seconds
int lastMax = 0;                  // have to also store the max to avoid +-255 values


// when using ATtiny85 on 16MHz, serial pin is 2, not 3!!!

void setup() {
  // real code
  pinMode(2, INPUT); // used for step interrupt
  pinMode(5, INPUT); // used for step direction
  // Setting the PWM pins as output
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  // Set the PWM frequencies
  TCCR0A = TCCR0A & 0b11111000 | 0x01;
  TCCR0B = TCCR0B & 0b11111000 | 0x01;

  // interrupts
  GIMSK = 0b00100000;    // turns on pin change interrupts
  PCMSK = 0b00011100;    // turn on interrupts on pins PB3 and PB4
  sei();

  getMinimal();

  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);
  myPID.SetControllerDirection((moveCoef > 0)?DIRECT:REVERSE);
  myPID.SetOutputLimits(-255, 255);
}

void loop() {
  curTime = millis();
  input = encoder0Pos;
  setpoint = target1;
  myPID.Compute();

  pwmOut(output);

  /* Motor safe code */
  if (abs(output) == 255) {
    if (lastSafeCheck == 0) {
      lastSafeCheck = curTime;
      lastEncPos = encoder0Pos;
    } else {
      if (lastEncPos != encoder0Pos) {
        lastSafeCheck = 0;
      } else {
        if (curTime - motorSafe > lastSafeCheck) {
          // we have to protect the motor - looks like even with max output we are not moving
          // we will set the target1 to current position to stop output
          target1 = encoder0Pos - (output / 255 * 5);
          lastEncPos = 0;
          lastSafeCheck = 0;
        }
      }
    }
  } else {
    lastSafeCheck = 0;
    lastEncPos = 0;
  }
}


// interrupt handling - have to select which triggered!!!
const int QEM [16] = {0, -1, 1, 2, 1, 0, 2, -1, -1, 2, 0, 1, 2, 1, -1, 0}; // Quadrature Encoder Matrix

ISR(PCINT0_vect)
{
  byte oldInput, newInput, oldPort;
  // reading port status and storing it
  oldPort = newPort;
  newPort = PINB;

  // determining if encoder position changed
  newInput = (newPort & 0b00011000) >> 3;
  oldInput = (oldPort & 0b00011000) >> 3;
  if (newInput != oldInput) {
    // we have change in Encoder position!
    encoder0Pos += QEM[oldInput * 4 + newInput];
  }

  // determining if step input occured
  // let's check only the step pin,
  // we'll care about direction later
  newInput = newPort & 0b0100;
  oldInput = oldPort & 0b0100;

  // we have to have change comparing to previous state
  // and we need raising change on input value for step
  // here I do 10 steps per impusle!!!
  if ((newInput != 0) && (oldInput == 0)) {
    // we have step!
    #ifndef stepKoef
    target1 += (newPort & 0b100000) ? -1 : 1;
    #else
    target1 += (newPort & 0b100000) ? -5 : 5;
    #endif
  }
}

void pwmOut(int out) {
  if (out < 0) {
    analogWrite(M1, 0);
    analogWrite(M2, abs(out));
  }
  else {
    analogWrite(M1, abs(out));
    analogWrite(M2, 0);
  }
}


void getMinimal() {

  // goal is to:
  // 1. measure minimal speed
  // 2.a if we have no encoder input, perhaps we are at the end
  //     of possible movement track and we need to reverse
  // 2.b if we have no encoder input after reverse, we need to
  //     stop everything - either the motor is disconnected
  //     or the encoder is and whatever we send can cause damage
  // 3. verify encoder direction - to ensure positive movement
  //    of encoder input is also positive direction of motor
  // 4. if we did not have a movement, the motor is disconnected
  //    and we are probably in a setup mode - we need to turn on
  //    the serial on respective pins

  // first, let's find minimum moving speed
  int minSpeed = minSpd(1);

  // did we moved?
  // if we did not move, we have to try the other way
  // perhaps we were at the end of the path
  // and we need to go back
  if (encoder0Pos == 0) minSpeed = minSpd(-1);

  // stop the motor
  pwmOut(0);

  // did we moved now?
  // if we did not, we have to end - probably we need to turn on the serial
  // port to debug, so we do it and quit the setup routine
  if (encoder0Pos == 0) {
    // Fatal error
    // no input from encoder
    // will halt
    while (1) {
      pwmOut(125);
      delay(200);
      pwmOut(-125);
      delay(200);
    }
  }

  // we moved, variable i has the minimum moving speed
  // we need to verify if the encoder is aligned with
  // the direction of the motor
  moveCoef = (encoder0Pos / abs(encoder0Pos)) * (minSpeed / abs(minSpeed));

  #ifdef minSearch
  // now move to minimum position
  minSpeed = abs(minSpeed) + (255 - abs(minSpeed)) / 3;
  long encoderOld = 0;
    pwmOut(-minSpeed * (moveCoef));
    while (encoderOld != encoder0Pos) {
      encoderOld = encoder0Pos;
      delay(50);
    }
    pwmOut(0);
    encoder0Pos = -75;
  #endif
}

int minSpd(int dirSpd) {
  int minSpeed = 0;
  while (( abs(minSpeed) <= 255) && (abs(encoder0Pos) < 5)) {
    pwmOut(minSpeed * dirSpd);
    delay(10);
    minSpeed += 5;
  }
  return (minSpeed * dirSpd);
}
