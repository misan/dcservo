/* 
   Miguel Sanchez 2106
   This program uses an Arduino Pro Micro variant for a closed-loop control of a DC-motor.
   Motor motion is detected by a quadrature encoder.
   Two inputs named STEP and DIR allow changing the target position.
   Serial port prints current position and target position every second.
   Serial input can be used to feed a new location for the servo (no CR LF).

   Pins used:
   Digital inputs 2 & 3 are connected to the two encoder signals (AB).
   Digital input 0 is the STEP input.
   Analog input A0 is the DIR input.
   Digital outputs 6 & 7 control the direction outputs for the motor (I am using half TB6612FNG here).
   Digital output 9 is PWM motor control


   Please note PID gains kp, ki, kd need to be tuned to each different setup.
*/
// for 16-bit PWM on pin 9
#define PWM OCR1A


#include <EEPROM.h>
#include <PID_v1.h>
#define encoder0PinA  3 // PD0; 
#define encoder0PinB  2 // PD1;

#define M1            6  // No motor's PWM outputs
#define M2            7  // just set the direction

int pos[1000]; int p = 0;

double kp = 3, ki = 0, kd = 0.0;
double feed = 50;
double input = 0, output = 0, setpoint = 0;
PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);
// speed loop
double motor = 0, setspeed = 100, vel = 0;
double vkp = 1, vki = 1, vkd = 0;
PID speed(&vel, &motor, &setspeed, vkp, vki, vkd, DIRECT);
volatile long encoder0Pos = 0;
boolean auto1 = false, auto2 = false, counting = false;
long previousMillis = 0;        // will store last time LED was updated

long target1 = 0; // destination location at any moment

//for motor control ramps 1.4
bool newStep = false;
bool oldStep = false;
bool dir = false;
byte skip = 0;

float accel = 100.0; // desired acceleration in mm/s^2



void setup() {
  pinMode(encoder0PinA, INPUT);
  pinMode(encoder0PinB, INPUT);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  attachInterrupt(0, encoderInt, CHANGE);  // encoder pin on interrupt 0 - pin 3
  attachInterrupt(1, encoderInt, CHANGE);  // encoder pin on interrupt 1 - pin 2

  pinMode(9, OUTPUT);
  /*
    // Set timer 1 to 16-bit Fast PWM
    ICR1 = 0xFFFF;
    TCCR1A = 0b10101010;
    TCCR1B = 0b00011001;
    PWM=0; */
  //pinMode(0,OUTPUT); // eliminar
  attachInterrupt(2, countStep      , RISING);  // step  input on interrupt 2 - pin 0
  TCCR1B = TCCR1B & 0b11111000 | 1; // set 31Kh PWM
  Serial.begin (115200);
  help();
  recoverPIDfromEEPROM();
  //Setup the pid
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);
  myPID.SetOutputLimits(-255, 255);

  speed.SetMode(AUTOMATIC);
  speed.SetSampleTime(1);
  speed.SetOutputLimits(-255, 255);
}

void loop() {
  vel =  encoder0Pos - input;
  input = encoder0Pos;
  setpoint = target1;
  while(!myPID.Compute()); // wait till PID is actually computed
  setspeed = output;
  if (Serial.available()) process_line(); // it may induce a glitch to move motion, so use it sparingly
  if (auto1) if (millis() % 1000 == 0) trapezoidal(random(6000)); //target1 = random(2000); // that was for self test with no input from main controller
  if (auto2) if (millis() % 1000 == 0) printPos();
  //if(counting && abs(input-target1)<15) counting=false;
  if (  speed.Compute() && counting ) { // only sample when PID updates
    pos[p] = encoder0Pos;
    if (p < 999) p++;
    else counting = false;
  }
    pwmOut(motor);
}

/*
  void pwmOut(int out) {
   if(out<0) { analogWrite(M1,0); analogWrite(M2,abs(out)); }
   else { analogWrite(M2,0); analogWrite(M1,abs(out)); }
  }
*/
void pwmOut(int out) {
  if (out > 0) {
    digitalWrite(M1, 0);
    digitalWrite(M2, 1);
  }
  else      {
    digitalWrite(M1, 1);
    digitalWrite(M2, 0);
  }
  analogWrite(9, abs(out));
  //PWM = out;
}

const int QEM [16] = {0, -1, 1, 2, 1, 0, 2, -1, -1, 2, 0, 1, 2, 1, -1, 0}; // Quadrature Encoder Matrix
static unsigned char New, Old;


void encoderInt() { // handle pin change interrupt for D2
  Old = New;
  New = PIND & 3; //(PINB & 1 )+ ((PIND & 4) >> 1); //
  encoder0Pos += QEM [Old * 4 + New];
}


void countStep() {
  if (PINF & B10000000) target1--;  // pin A0 represents direction == PF7 en Pro Micro
  else target1++;
}

void process_line() {
  char cmd = Serial.read();
  if (cmd > 'Z') cmd -= 32;
  switch (cmd) {
    case 'P': kp = Serial.parseFloat(); myPID.SetTunings(kp, ki, kd); break;
    case 'D': kd = Serial.parseFloat(); myPID.SetTunings(kp, ki, kd); break;
    case 'I': ki = Serial.parseFloat(); myPID.SetTunings(kp, ki, kd); break;
    case '?': printPos(); break;
    case 'X': target1 = Serial.parseInt(); counting = true; for (int i = 0; i < p; i++) pos[i] = 0; p = 0; break;
    case 'T': auto1 = !auto1; break;
    case 'A': auto2 = !auto2; break;
    case 'Q': Serial.print("P="); Serial.print(kp); Serial.print(" I="); Serial.print(ki); Serial.print(" D="); Serial.print(kd); Serial.print(" VP="); Serial.print(vkp); Serial.print(" VI="); Serial.println(vki); break;
    case 'H': help(); break;
    case 'W': writetoEEPROM(); break;
    case 'K': eedump(); break;
    case 'R': recoverPIDfromEEPROM() ; break;
    case 'S': for (int i = 0; i < p; i++) Serial.println(pos[i]); break;
    case 'Z': detachInterrupt(2); break; // from then on, ignore step pulses (good for tests)
    case 'F': feed = Serial.parseFloat(); break;
    case 'V': vkp = Serial.parseFloat(); speed.SetTunings(vkp, vki, vkd); break;
    case 'G': vki = Serial.parseFloat(); speed.SetTunings(vkp, vki, vkd); break;
    case 'Y': counting = true; for (int i = 0; i < p; i++) pos[i] = 0; p = 0; trapezoidal(Serial.parseInt()); break; // performs a trapezoidal move
    case '@': accel = Serial.parseFloat(); break;
  }
  while (Serial.read() != 10); // dump extra characters till LF is seen (you can use CRLF or just LF)
}

void printPos() {
  Serial.print(F("Position=")); Serial.print(encoder0Pos); Serial.print(F(" PID_output=")); Serial.print(output); Serial.print(F(" Target=")); Serial.println(setpoint);
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
  Serial.println(F("A will toggle on/off showing regulator status every second"));
  Serial.println(F("F sets desired motion speed"));
  Serial.println(F("V sets speed proportional gain"));
  Serial.println(F("G sets speed integral gain"));
  Serial.println(F("Y123.34 it is like X but using trapezoidal motion"));
  Serial.println(F("@123.34 sets [trapezoidal] acceleration"));
  Serial.println(F("Z disables STEP input"));
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

void trapezoidal(int destination) { // it will use acceleration and feed values to restrict the motion following a trapezoidal pattern
  long a1 = millis();
  int distance = destination - setpoint; // if positive go +x
  int finalPoint = setpoint + distance;
  boolean dirPos = true;
  if (distance < 0) {
    distance = -distance;  // me quedo con el valor absoluto del movimiento
    dirPos = false;
  }
  float xm = feed * feed / accel;
  float t1, t2;
  if (distance <= xm) t1 = t2 = sqrt(distance / accel); // triangular
  else { // trapezoidal
    t1 = sqrt(xm / accel); // t1 = end of accel
    t2 = (distance - xm) / feed + t1; // t2 = end of coasting
  }
  // Ok, I know what to do next, so let's perform the actual motion
  float t = 0, spd = 0.0;
  float dt = 1e-3;
  float da = accel * dt;
  float covered = setpoint;
  float maxt = t1 + t2;
  while (t < maxt) {
    t += dt;
    if (t < t1) spd += da; else if (t >= t2) spd -= da;
    if ( dirPos ) covered += spd * dt; else covered -= spd * dt; // calculate new target position
    //vel =  encoder0Pos - input;
    input = encoder0Pos;
    setpoint = covered;
    while(!myPID.Compute()); // espero a que termine el cálculo
    setspeed = output;
    //speed.Compute();
    pwmOut(output );
    //digitalWrite(0,1-digitalRead(0));; just for time tracing purposes
    // record data for S command
    if (counting  ) {
      pos[p] = encoder0Pos;
      if (p < 999) p++;
      else counting = false;
    }
  }
  //Serial.print(millis() - a1); Serial.print(F("  Err=")); Serial.println(encoder0Pos - covered);
  target1 = covered;
}

