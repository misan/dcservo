/*
   This program uses an Arduino for a closed-loop control of a DC-motor.
   Motor motion is detected by a quadrature encoder.
   Two inputs named STEP and DIR allow changing the target position.
   Serial port prints current position and target position every second.
   Serial input can be used to feed a new location for the servo (no CR LF).

   Pins used:
   Digital inputs 2 & 8 are connected to the two encoder signals (AB).
   Digital input 3 is the STEP input.
   Analog input 0 is the DIR input.
   Digital outputs 9 & 10 control the PWM outputs for the motor (I am using half L298 here).
   Please note PID gains kp, ki, kd need to be tuned to each different setup.
*/
#include <EEPROM.h>
#include <PID_v1.h>

#define encoder0PinA  2 // PD2; 
#define encoder0PinB  8  // PC0;
#define M1            9
#define M2            10  // motor's PWM outputs

int pos[500]; int p = 0;
volatile int backlash = 0; // grabbed from SingularitySurfer's git

double kp = 16, ki = 16, kd = 0.10;
double input = 0, output = 0, setpoint = 0;

PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);

volatile long encoder0Pos = 0;
volatile int moveCoef = 1;
boolean setupDone = false;

boolean auto1 = false, auto2 = false, counting = false;
long previousMillis = 0;        // will store last time LED was updated
long lastEncPos = 0;          // will store the target value when last max output was measured
unsigned long curTime = 0UL;  // will store current time to avoid multiple millis() calls
unsigned long lastSafeCheck = 0;  // will store last value when 255 output was measured
unsigned long motorSafe = 1000UL; // will store the interval to protect the motor - 2seconds
int lastMax = 0;              // have to also store the max to avoid +-255 values

long target1 = 0; // destination location at any moment

//for motor control ramps 1.4
bool newStep = false;
bool oldStep = false;
bool dir = false;
int skip = 0;

// Install Pin change interrupt for a pin, can be called multiple times
void pciSetup(byte pin)
{
  *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
  PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
  PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}

void setup() {
  pinMode(encoder0PinA, INPUT);
  pinMode(encoder0PinB, INPUT);
  pciSetup(encoder0PinB);
  attachInterrupt(0, encoderInt, CHANGE);  // encoder pin on interrupt 0 - pin 2
  attachInterrupt(1, countStep , RISING);  // step  input on interrupt 1 - pin 3
  TCCR1B = TCCR1B & 0b11111000 | 1; // set 31Kh PWM
  Serial.begin (115200);
  help();
  recoverPIDfromEEPROM();

  Serial.println("Measuring minimal speed");
  // first, let's find minimum moving speed
  int minSpeed = 0;
  while (( minSpeed <= 255) && (abs(encoder0Pos) < 5)) {
    pwmOut(minSpeed);
    delay(10);
    minSpeed += 5;
  }

  // did we moved?
  if (encoder0Pos == 0) {
    // we did not move, we have to try the other way
    // perhaps we were at the end of the path
    // and we need to go back
    minSpeed = 0;
    while (( minSpeed >= -255) && (abs(encoder0Pos) < 5)) {
      pwmOut(minSpeed);
      delay(10);
      minSpeed -= 5;
    }
  }

  // stop the motor
  pwmOut(0);

  // did we moved now?
  // if we did not, we have to end - either the encoder is not working
  // or we don't have enough power to move
  // therefore we will freeze
  if (encoder0Pos == 0) {
    Serial.println("We have a problem - no movement!!!");
    while (1) delay(1);
  }

  Serial.print("Minimal speed: ");
  Serial.println(minSpeed);
  Serial.print("Encoder position: ");
  Serial.println(encoder0Pos);
  
  // we moved, variable i has the minimum moving speed
  // we need to verify if the encoder is aligned with
  // the direction of the motor
  moveCoef = (encoder0Pos / abs(encoder0Pos)) * (minSpeed / abs(minSpeed));

  Serial.print("Movement coeficient: ");
  Serial.println(moveCoef);

  // to ensure we will move
  // we will increase the minimal moving speed
  // by 25% of the difference to maximum speed
  minSpeed = abs(minSpeed) + (255 - abs(minSpeed)) / 5;

  Serial.print("Adjusted movement speed: ");
  Serial.println(minSpeed);

  // now we have reasonable speed to move but not break things
  // we will find minimum and maximum
  // move in negative direction by minimum speed
  // until encoder stops
  long encoderOld = 0;
  pwmOut(-minSpeed);
  while (encoderOld != encoder0Pos) {
    encoderOld = encoder0Pos;
    delay(50);
  }
  pwmOut(0);

  Serial.print("Measured minimum before reset to 0: ");
  Serial.println(encoder0Pos);

  encoder0Pos = 0;
  pwmOut(minSpeed);
  while (encoderOld != encoder0Pos) {
    encoderOld = encoder0Pos;
    delay(50);
  }
  pwmOut(0);

  Serial.print("Measured maximum: ");
  Serial.println(encoder0Pos);
  
  //Setup the pid
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);
  myPID.SetOutputLimits(-255, 255);
}

void loop() {
  curTime = millis();
  input = encoder0Pos;
  setpoint = target1;
  myPID.Compute();
  if (Serial.available()) process_line(); // it may induce a glitch to move motion, so use it sparingly
  pwmOut(output);
  if (auto1) if (curTime % 3000 == 0) target1 = random(2000); // that was for self test with no input from main controller
  if (auto2) if (curTime % 1000 == 0) printPos();
  //if(counting && abs(input-target1)<15) counting=false;
  if (counting &&  (skip++ % 10) == 0 ) {
    pos[p] = encoder0Pos;
    if (p < 499) p++;
    else counting = false;
  }
  /* Motor safe code */
  if ((output == 255) || (output == -255)) {
    if (lastSafeCheck == 0) {
      lastSafeCheck = curTime;
      lastEncPos = encoder0Pos;
    } else {
      if (lastEncPos != encoder0Pos) {
        lastSafeCheck = 0;
      } else {
        if (curTime - motorSafe > lastSafeCheck) {
          // we have to protect the motor - looks like even with max output we are not moving
          // we will set the target to current position to stop output
          Serial.println(F(" Will decrease output!!!"));
          target1 = encoder0Pos - (output / 255 * 5);
          lastEncPos = 0;
          lastSafeCheck = curTime;
        }
      }
    }
  } else {
    lastSafeCheck = 0;
    lastEncPos = 0;
  }

}

void pwmOut(int out) {
  out = out * moveCoef;
  if (out < 0) {
    analogWrite(M1, 0);
    analogWrite(M2, abs(out));
  }
  else {
    analogWrite(M2, 0);
    analogWrite(M1, abs(out));
  }
}

const int QEM [16] = {0, -1, 1, 2, 1, 0, 2, -1, -1, 2, 0, 1, 2, 1, -1, 0}; // Quadrature Encoder Matrix

static unsigned char New, Old;
ISR (PCINT0_vect) { // handle pin change interrupt for D8
  Old = New;
  New = (PINB & 1 ) + ((PIND & 4) >> 1); //
  encoder0Pos += QEM [Old * 4 + New];
}

void encoderInt() { // handle pin change interrupt for D2
  Old = New;
  New = (PINB & 1 ) + ((PIND & 4) >> 1); //
  encoder0Pos += QEM [Old * 4 + New];
}


void countStep() {
  if (PINC & B0000001) {
    target1--;
  }
  else {
    target1++;
  }
} // pin A0 represents direction


void process_line() {
  char cmd = Serial.read();
  if (cmd > 'Z') cmd -= 32;
  switch (cmd) {
    case 'P': kp = Serial.parseFloat(); myPID.SetTunings(kp, ki, kd); break;
    case 'D': kd = Serial.parseFloat(); myPID.SetTunings(kp, ki, kd); break;
    case 'I': ki = Serial.parseFloat(); myPID.SetTunings(kp, ki, kd); break;
    case '?': printPos(); break;
    case 'X': target1 = Serial.parseInt(); p = 0; counting = true; for (int i = 0; i < 500; i++) pos[i] = 0; break;
    case 'T': auto1 = !auto1; break;
    case 'A': auto2 = !auto2; break;
    case 'Q': Serial.print("P="); Serial.print(kp); Serial.print(" I="); Serial.print(ki); Serial.print(" D="); Serial.println(kd); break;
    case 'H': help(); break;
    case 'W': writetoEEPROM(); break;
    case 'K': eedump(); break;
    case 'R': recoverPIDfromEEPROM() ; break;
    case 'S': for (int i = 0; i < p; i++) Serial.println(pos[i]); break;
    case 'B': backlash = Serial.parseFloat();
  }
  while (Serial.read() != 10); // dump extra characters till LF is seen (you can use CRLF or just LF)
}

void printPos() {
  Serial.print(F("Position="));
  Serial.print(encoder0Pos);
  Serial.print(F(" PID_output="));
  Serial.print(output);
  Serial.print(F(" Target="));
  Serial.print(setpoint);
  Serial.print(F(" LastCheck="));
  Serial.print(lastSafeCheck);
  Serial.print(F(" LastEnc="));
  Serial.print(lastEncPos);
  Serial.print(F(" Coeficient="));
  Serial.println(moveCoef);

}

void help() {
  Serial.println(F("\nPID DC motor controller and stepper interface emulator"));
  Serial.println(F("by misan"));
  Serial.println(F("Available serial commands: (lines end with CRLF or LF)"));
  Serial.println(F("P123.34 sets proportional term to 123.34"));
  Serial.println(F("I123.34 sets integral term to 123.34"));
  Serial.println(F("D123.34 sets derivative term to 123.34"));
  Serial.println(F("? prints out current encoder, output and setpoint values"));
  Serial.println(F("X123 sets the target destination for the motor to 123 encoder pulses"));
  Serial.println(F("T will start a sequence of random destinations (between 0 and 2000) every 3 seconds. T again will disable that"));
  Serial.println(F("Q will print out the current values of P, I and D parameters"));
  Serial.println(F("W will store current values of P, I and D parameters into EEPROM"));
  Serial.println(F("H will print this help message again"));
  Serial.println(F("A will toggle on/off showing regulator status every second\n"));
  Serial.println(F("B123 will set backlash compensation to 123 steps per change in direction"));
  Serial.print(F("  Backlash Value: ")); Serial.println(backlash);
}

void writetoEEPROM() { // keep PID set values in EEPROM so they are kept when arduino goes off
  eeput(kp, 0);
  eeput(ki, 4);
  eeput(kd, 8);
  double cks = 0;
  for (int i = 0; i < 12; i++) cks += EEPROM.read(i);
  eeput(cks, 12);
  Serial.println("\nPID values stored to EEPROM");
  //Serial.println(cks);
}

void recoverPIDfromEEPROM() {
  double cks = 0;
  double cksEE;
  for (int i = 0; i < 12; i++) cks += EEPROM.read(i);
  cksEE = eeget(12);
  //Serial.println(cks);
  if (cks == cksEE) {
    Serial.println(F("*** Found PID values on EEPROM"));
    kp = eeget(0);
    ki = eeget(4);
    kd = eeget(8);
    myPID.SetTunings(kp, ki, kd);
  }
  else Serial.println(F("*** Bad checksum"));
}

void eeput(double value, int dir) { // Snow Leopard keeps me grounded to 1.0.6 Arduino, so I have to do this :-(
  char * addr = (char * ) &value;
  for (int i = dir; i < dir + 4; i++)  EEPROM.write(i, addr[i - dir]);
}

double eeget(int dir) { // Snow Leopard keeps me grounded to 1.0.6 Arduino, so I have to do this :-(
  double value;
  char * addr = (char * ) &value;
  for (int i = dir; i < dir + 4; i++) addr[i - dir] = EEPROM.read(i);
  return value;
}

void eedump() {
  for (int i = 0; i < 16; i++) {
    Serial.print(EEPROM.read(i), HEX);
    Serial.print(" ");
  } Serial.println();
}
