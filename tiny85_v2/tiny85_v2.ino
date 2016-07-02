/*
 * Goal: Have DC motor with encoder
 * driven by ATtiny85 and Step and Direction
 * from RAMPS
 * However, when starting, I want to determine a "DEBUG"
 * condition and wnable serial port to set PID and parameters
 * like enable (minimum and maximum search.
 *
 * Tiny setup:
 * 1 - PB5 - Encoder A
 * 2 - PB3 - Encoder B
 * 3 - PB4 - DC PWM
 * 4 - GND
 * 5 - PB0 - Step Impulse / Serial RX
 * 6 - PB1 - Step Direction / Serial TX
 * 7 - PB2 - DC direction
 * 8 - VCC
 */
#include "avr/interrupt.h"
#include <PID_v1.h>

// please adjust these 3 values for your motor!!!
double kp = 50, ki = 4, kd = 0.10;
double input = 0, output = 0, setpoint = 0;
PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);
// I'll use SetControllerDirection to set the direction later

boolean auto2 = true;

unsigned long previousMillis = 0; // will store last time LED was updated
long lastEncPos = 0;              // will store the encoded value when last max output was measured
unsigned long curTime = 0UL;      // will store current time to avoid multiple millis() calls
unsigned long lastSafeCheck = 0;  // will store last value when 255 output was measured
unsigned long motorSafe = 4000UL; // will store the interval to protect the motor - 4seconds
int lastMax = 0;                  // have to also store the max to avoid +-255 values
boolean debug = false;

volatile int encoder0Pos = 0;
volatile long target1 = 0; // destination location at any moment
volatile int moveCoef = 1;
static byte newPort;

// Encoder table - get the result based on previous and current encoder output
const int QEM [16] = {0, -1, 1, 2, 1, 0, 2, -1, -1, 2, 0, 1, 2, 1, -1, 0}; // Quadrature Encoder Matrix

void setup() {
  setPWM();
  /*
    pinMode(0, INPUT); // used for step interrupt
    pinMode(1, INPUT); // used for step direction
    if (digitalRead(0) && digitalRead(1)) {
    */
  // we have debug condition (both step and dir = 1 during setup
  // we will start serial on these pins instead of listening to interrupts
  Serial.begin(115200);
  Serial.println("Setup in debug mode!");
  //delay(5000);      // wait 5s to disable the startup condition
  debug = true;
  //}

  // interrupts
  GIMSK = 0b00100000;    // turns on pin change interrupts
  PCMSK = 0b00010100;    // turn on interrupts on pins PB3 and PB4
  if (!debug) PCMSK |= 0b1; // if we are not debuging, we need step input, too
  sei();

  //getMinimal();

  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);
  //myPID.SetControllerDirection((moveCoef > 0) ? DIRECT : REVERSE);
  myPID.SetOutputLimits(-255, 255);

  if (debug) Serial.println("Setup done");
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

  if (auto2) if (curTime % 1000 == 0) printPos();
  if (Serial.available()) processLine();
}


ISR(PCINT0_vect)
{
  byte oldInput, newInput, oldPort;
  // reading port status and storing it
  oldPort = newPort;
  newPort = PINB;

  // determining if encoder position changed
  // encoder connected to PB5 and PB3
  newInput = (newPort & 0b00101000) >> 3;
  oldInput = (oldPort & 0b00101000) >> 3;
  if (newInput != oldInput) {
    // we have change in Encoder position!
    // the math here is to fix the encoder position
    // as I am interested in 3rd and 1st bit, I need to remove
    // the second one
    newInput = newInput >> 2 | (newInput & 1);
    encoder0Pos += QEM[(oldInput >> 2 | (oldInput & 1)) * 4 + (newInput >> 2 | (newInput & 1))];
  }

  // determining if step input occured
  // let's check only the step pin,
  // we'll care about direction later
  newInput = newPort & 0b01;
  oldInput = oldPort & 0b01;

  // we have to have change comparing to previous state
  // and we need raising change on input value for step
  // the direction of step is determined by PB1
  if ((newInput != 0) && (oldInput == 0)) {
    // we have step!
    target1 += (newPort & 0b10) ? -1 : 1;
  }
}


void pwmOut(int out) {
  if (out < 0) {
    digitalWrite(2, HIGH);
    OCR1B = 255 + out;
  }
  else {
    digitalWrite(2, LOW);
    OCR1B = out;
  }
}

void setPWM() {
  // PWM setup according
  // http://matt16060936.blogspot.ch/2012/04/attiny-pwm.html
  DDRB |= 1 << DDB4; // set the PB4 as output
  DDRB |= 1 << DDB2; // set the PB2 as output (direction pin)
  TCCR0A = 1 << COM0A0 | 2 << COM0B0 | 3 << WGM00;
  // set the timers COM0A0 for enabling
  // timers and the rest for timer B
  // without COM0A0 the Timer1 will not work
  TCCR1 = 0 << PWM1A | 0 << COM1A0 | 1 << CS10;
  // this tells not to use pins OC0B and OC1A
  // and use no prescaler
  GTCCR = 1 << PWM1B | 2 << COM1B0;
  // enable OC1B (PB4)
  // and clears it to bottom
  // I'll use OCR1B = x to set the PWM value
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
    Serial.println("Err");
    if (!debug) while (1) delay(1);
  }

  // we moved, variable i has the minimum moving speed
  // we need to verify if the encoder is aligned with
  // the direction of the motor
  moveCoef = (encoder0Pos / abs(encoder0Pos)) * (minSpeed / abs(minSpeed));
}

int minSpd(int dirSpd) {
  int minSpeed = 0;
  while (( abs(minSpeed) <= 255) & (abs(encoder0Pos) < 5)) {
    pwmOut(minSpeed * dirSpd);
    delay(10);
    minSpeed += 5;
  }
  return (minSpeed * dirSpd);
}


void printPos() {
  Serial.print("Pos=");
  Serial.print(encoder0Pos);
  Serial.print(" PID=");
  Serial.print(output);
  Serial.print(" Target=");
  Serial.println(setpoint);
}

void processLine() {
  char cmd = Serial.read();
  if (cmd > 'Z') cmd -= 32;
  switch (cmd) {
    case 'P': kp = parseFloat(); myPID.SetTunings(kp, ki, kd); break;
    case 'D': kd = parseFloat(); myPID.SetTunings(kp, ki, kd); break;
    case 'I': ki = parseFloat(); myPID.SetTunings(kp, ki, kd); break;
    case '?': printPos(); break;
    //case 'H': help(); break;
   //    case 'X': target1 = parseInt(); p = 0; counting = true; for (int i = 0; i < 500; i++) pos[i] = 0; break;
    case 'X': target1 = parseInt(); break;
    case 'Q': Serial.print("P="); Serial.print(kp); Serial.print(" I="); Serial.print(ki); Serial.print(" D="); Serial.println(kd); break;
  }
  while (Serial.read() != 10); // dump extra characters till LF is seen (you can use CRLF or just LF)
}

float parseFloat() {
  float returnVal = 0;
  int isDecimal = 0;

  while (Serial.available() > 0) {
    char serialChar = Serial.read();
    if ((serialChar >= '0') & (serialChar <= '9')) {
      returnVal *= 10;
      if (isDecimal > 0) isDecimal *= 10;
      returnVal += int(serialChar) - int('0');
    }
    if (serialChar == '.') isDecimal = 1;
  }
  if (isDecimal > 0) returnVal /= isDecimal;
  Serial.print("Fl:");
  Serial.println(returnVal);
  return (returnVal);
}

int parseInt() {
  int returnVal = 0;

  while (Serial.available() > 0) {
    char serialChar = Serial.read();
    if ((serialChar >= '0') & (serialChar <= '9')) {
      returnVal *= 10;
      returnVal += int(serialChar) - int('0');
    }
  }
  Serial.print("Int:");
  Serial.println(returnVal);
  return (returnVal);
}
