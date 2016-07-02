/*
 * Miguel Sanchez 2106
   Mauro Manco 2016 Porting on ESP8266

   This program uses an Arduino Pro Micro variant for a closed-loop control of a DC-motor.
   Motor motion is detected by a quadrature encoder.
   Two inputs named STEP and DIR allow changing the target position.
   Serial port prints current position and target position every second.
   Serial input can be used to feed a new location for the servo (no CR LF).
   Please note PID gains kp, ki, kd need to be tuned to each different setup.
*/

#include <EEPROM.h>
#include <PID_v1.h>
#include <Wire.h>         // support for I2C encoder


const int Step = 14;
const int M1 = 9;
const int M2 = 10;
const int DIR = 4;
const int PWM_MOT = 15;

byte pos[1000]; int p = 0;
byte maxPower = 255;
double kp = 12, ki = 4, kd = 0.01;
double input = 0, output = 0, setpoint = 0;
PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);
//volatile long encoder0Pos = 0;
long pos1;

boolean auto1 = false, auto2 = true, counting = false;
long previousMillis = 0;        // will store last time LED was updated

long target1 = 0; // destination location at any moment

//for motor control ramps 1.4
byte skip = 0;
int angle, diff;
double before = 0;


void setup() {
  Serial.begin (115200);
  pinMode(13, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);

  TCCR1B = TCCR1B & 0b11111000 | 1; // set 31Kh PWM

  recoverPIDfromEEPROM();
  //Setup the pid
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);
  myPID.SetOutputLimits((-1) * maxPower, maxPower);

  Wire.begin(); // start I2C driver code
  help();
}

void loop() {
  angle = readTwoBytes() >> 2; //analogRead(A0); // encoder0Pos;
  // process encoder rollover
  diff = angle - before;
  if (diff < -768) pos1 += 1024;
  else if (diff > 768) pos1 -= 1024;
  before = angle;
  input = pos1 + angle;

  //input = encoder0Pos;
  setpoint = target1;
  myPID.Compute();
  if (Serial.available()) process_line();
  pwmOut(output);
  if (auto1) if (millis() % 3000 == 0) target1 = random(2000); // that was for self test with no input from main controller
  if (auto2) if (millis() % 1000 == 0) printPos();
  if (counting && abs(input - target1) < 15) counting = false;
  if (counting &&  (skip++ % 2) == 0 ) {
    pos[p] = input;
    if (p < 999) p++;
    else counting = false;
  }
}

word readTwoBytes()
{
  word retVal = -1;

  /* Read Low Byte */
  Wire.beginTransmission(0x36);
  Wire.write(0x0d);
  Wire.endTransmission();
  Wire.requestFrom(0x36, 1);
  while (Wire.available() == 0);
  int low = Wire.read();

  /* Read High Byte */
  Wire.beginTransmission(0x36);
  Wire.write(0x0c);
  Wire.endTransmission();
  Wire.requestFrom(0x36, 1);

  while (Wire.available() == 0);

  word high = Wire.read();

  high = high << 8;
  retVal = high | low;

  return retVal;
}


void pwmOut(int out) {
  if (out > 0) {
    digitalWrite(M1, 0);
    analogWrite(M2, out);
  }
  else      {
    analogWrite(M1, -out);
    digitalWrite(M2, 0);
  }
}

void printPos() {
  Serial.print(F("Position=")); Serial.print(input); Serial.print(F(" PID_output=")); Serial.print(output); Serial.print(F(" Target=")); Serial.print(setpoint); Serial.print(F(" Before=")); Serial.print(before); Serial.print(F(" Angle=")); Serial.println(angle);
}

void help() {
  Serial.println(F("\nPID DC motor controller and stepper interface emulator"));
  Serial.println(F("by misan - porting cured by Exilaus"));
  Serial.println(F("Available serial commands: (lines end with CRLF or LF)"));
  Serial.println(F("P123.34 sets proportional term to 123.34"));
  Serial.println(F("I123.34 sets integral term to 123.34"));
  Serial.println(F("D123.34 sets derivative term to 123.34"));
  Serial.println(F("M123 sets maximum motor power"));
  Serial.println(F("? prints out current encoder, output and setpoint values"));
  Serial.println(F("X123 sets the target destination for the motor to 123 encoder pulses"));
  Serial.println(F("T starts a sequence of random destinations (between 0 and 2000) every 3 seconds. T again will disable that"));
  Serial.println(F("Q prints out the current values of P, I and D parameters"));
  Serial.println(F("W stores current values of P, I and D parameters into EEPROM"));
  Serial.println(F("H prints this help message again"));
  Serial.println(F("A toggles on/off showing regulator status every second\n"));
}


void eeput(double value, int dir) { // Snow Leopard keeps me grounded to 1.0.6 Arduino, so I have to do this :-(
  char * addr = (char * ) &value;
  for (int i = dir; i < dir + 4; i++)  EEPROM.write(i, addr[i - dir]);
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


double eeget(int dir) { // Snow Leopard keeps me grounded to 1.0.6 Arduino, so I have to do this :-(
  double value;
  char * addr = (char * ) &value;
  for (int i = dir; i < dir + 4; i++) addr[i - dir] = EEPROM.read(i);
  return value;
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


void eedump() {
  for (int i = 0; i < 16; i++) {
    Serial.print(EEPROM.read(i), HEX);
    Serial.print(" ");
  } Serial.println();
}

void process_line() {
  char cmd = Serial.read();
  if (cmd > 'Z') cmd -= 32;
  switch (cmd) {
    case 'P': kp = Serial.parseFloat(); myPID.SetTunings(kp, ki, kd); break;
    case 'D': kd = Serial.parseFloat(); myPID.SetTunings(kp, ki, kd); break;
    case 'I': ki = Serial.parseFloat(); myPID.SetTunings(kp, ki, kd); break;
    case 'M': maxPower = Serial.parseInt(); myPID.SetOutputLimits((-1) * maxPower, maxPower); break;
    case '?': printPos(); break;
    case 'X': target1 = Serial.parseInt(); counting = true; for (int i = 0; i < p; i++) pos[i] = 0; p = 0; break;
    case 'T': auto1 = !auto1; break;
    case 'A': auto2 = !auto2; break;
    case 'Q': Serial.print("P="); Serial.print(kp); Serial.print(" I="); Serial.print(ki); Serial.print(" D="); Serial.println(kd); break;
    case 'H': help(); break;
    case 'W': writetoEEPROM(); break;
    case 'K': eedump(); break;
    case 'R': recoverPIDfromEEPROM() ; break;
    case 'S': for (int i = 0; i < p; i++) Serial.println(pos[i]); break;
  }
}
