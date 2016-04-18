/*
 * Temporary setting:
 * 0 - step input
 * 1 - step direction
 * 2 - serial debug
 * 3 - encoder A
 * 4 - encoder B
 *
 * Final setting:
 * 0 - motor PWM1
 * 1 - motor PWM2
 * 2 - step input
 * 3 - encoder A
 * 4 - encoder B
 * 5 - direction
 * 
 * To fix fuses after final non-debug program is uploaded:
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

// #define DEBUG

volatile int encoder0Pos = 0;
static byte newPort;

#define M1            0  // only PWM capable pins on Tiny is 0 and 1
#define M2            1

// please adjust these 3 values for your motor!!!
const double kp = 16, ki = 16, kd = 0.10;

double input = 0, output = 0, setpoint = 0;
PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);

boolean autoReport = true;
unsigned long previousMillis = 0; // will store last time LED was updated
long lastEncPos = 0;              // will store the encoded value when last max output was measured
unsigned long curTime = 0UL;      // will store current time to avoid multiple millis() calls
unsigned long lastSafeCheck = 0;  // will store last value when 255 output was measured
unsigned long motorSafe = 4000UL; // will store the interval to protect the motor - 4seconds
int lastMax = 0;                  // have to also store the max to avoid +-255 values

volatile long target1 = 0; // destination location at any moment

//for motor control ramps 1.4
bool newStep = false;
bool oldStep = false;
bool dir = false;


#if defined DEBUG
TinyDebugSerial mySerial = TinyDebugSerial();
#endif
// when using ATtiny85 on 16MHz, serial pin is 2, not 3!!!

void setup() {
  #if defined DEBUG
    // debug only - different pins
    mySerial.begin(115200);
    pinMode(0, INPUT);
    pinMode(1, INPUT);
  #else
    // real code
    pinMode(2, INPUT); // used for step interrupt
    pinMode(5, INPUT); // used for step direction
    // Setting the PWM pins as output
    pinMode(M1, OUTPUT);
    pinMode(M2, OUTPUT);
    // Set the PWM frequencies
    TCCR0A = TCCR0A & 0b11111000 | 0x01;
    TCCR0B = TCCR0B & 0b11111000 | 0x01;
  #endif

  // interrupts
  GIMSK = 0b00100000;    // turns on pin change interrupts
  #if defined DEBUG
    PCMSK = 0b00011001;    // turn on interrupts on pins PB3 and PB4
  #else
    PCMSK = 0b00011100;    // turn on interrupts on pins PB3 and PB4
  #endif
  sei();

  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);
  myPID.SetOutputLimits(-255, 255);

  #if defined DEBUG
    mySerial.println("Setup done");
  #endif
}

void loop() {
  curTime = millis();
  input = encoder0Pos;
  setpoint = target1;
  myPID.Compute();

  #if not defined DEBUG
    pwmOut(output);
  #else
    if (curTime % 1000 == 0) printPos();
  #endif

  #if not defined DEBUG
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
            #if defined DEBUG
              mySerial.println(F("Will decrease output!!!"));
            #endif
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
  #endif
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
  #if not defined DEBUG
    newInput = newPort & 0b0100;
    oldInput = oldPort & 0b0100;
  #else
    newInput = newPort & 0b01;
    oldInput = oldPort & 0b01;
  #endif
  
  // we have to have change comparing to previous state
  // and we need raising change on input value for step
  if ((newInput != 0) && (oldInput == 0)) {
    // we have step!
    #if not defined DEBUG
      target1 += (newPort & 0b100000)?-1:1;
    #else
      target1 += (newPort & 0b10)?-1:1;
    #endif
  }
}

#if not defined DEBUG
  void pwmOut(int out) {
    if (out < 0) {
      analogWrite(M1, 0);
      analogWrite(M2, abs(out));
    }
    else {
      analogWrite(M2, 0);
      analogWrite(M1, abs(out));
    }
  }
#else
  void printPos() {
    mySerial.print(F("Position="));
    mySerial.print(encoder0Pos);
    mySerial.print(F(" PID_output="));
    mySerial.print(output);
    mySerial.print(F(" stepoint="));
    mySerial.print(setpoint);
    mySerial.print(F(" target1="));
    mySerial.print(target1);
    mySerial.print(F(" LastCheck="));
    mySerial.print(lastSafeCheck);
    mySerial.print(F(" LastEnc="));
    mySerial.println(lastEncPos);
  }
#endif
