

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
#define PWM OCR1A
#include <EEPROM.h>
#include <PID_v1.h>
#define encoder0PinA  3 // PD0; 
#define encoder0PinB  2 // PD1;

#define ENDSTOP 5  // that is going to be an ouput signal to simulate an endstop

#define M1            6  // No motor's PWM outputs
#define M2            7  // just set the direction

byte pos[1000]; int p = 0;

double kp = 3, ki = 0, kd = 0.0;
double feed = 50000; // feedrate in counts/second
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

float accel = 100000.0; // desired acceleration in counts/s^2



void setup() {
  pinMode(encoder0PinA, INPUT);
  pinMode(encoder0PinB, INPUT);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(ENDSTOP, OUTPUT);  
  attachInterrupt(0, encoderInt, CHANGE);  // encoder pin on interrupt 0 - pin 3
  attachInterrupt(1, encoderInt, CHANGE);  // encoder pin on interrupt 1 - pin 2

  pinMode(9, OUTPUT);
  pinMode(0,INPUT_PULLUP); // eliminar
  attachInterrupt(2, countStep      , RISING);  // step  input on interrupt 2 - pin 0
  TCCR1B = TCCR1B & 0b11111000 | 1; // set 31Kh PWM
  Serial.begin (115200);
  while(!Serial); 
  help();
  recoverPIDfromEEPROM();
  //Setup the pid
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);
  myPID.SetOutputLimits(-255, 255);

  speed.SetMode(AUTOMATIC);
  speed.SetSampleTime(1);
  speed.SetOutputLimits(-255, 255);

  homing(); // 
}


void homing() {
long prevEncoder=encoder0Pos;
long error=0;
int power=0;

while(prevEncoder==encoder0Pos) pwmOut(power--); // increase power till we hardly have motion but not much more
if(power>128) pwmOut(120); // maybe blocked? , let's limit max power anyway
do {
   error=prevEncoder-encoder0Pos;
   delay(1);
} while(error!=prevEncoder-encoder0Pos);

pwmOut(0); 

  encoder0Pos=-70; // detected limit is now -70 to (if a soft limit is set like rubber motor would always try to push if zer0)
  target1=0;
}

int aut=-1;
void loop() {
  vel =  encoder0Pos - input;
  input = encoder0Pos;
  setpoint = target1;
  while(!myPID.Compute());
  setspeed = output;
  if (Serial.available()) process_line(); // it may induce a glitch to move motion, so use it sparingly
  if (auto1) if (millis() % 1000 == 0) switch(aut) {
      case 0: trapezoidal(random(14000)+1000); 
      case 1: senoidal(random(14000)+1000);
      case 2: cosenoidal(random(14000)+1000); //target1 = random(2000); // that was for self test with no input from main controller
  }
  if (auto2) if (millis() % 1000 == 0) printPos();
  //if(counting && abs(input-target1)<15) counting=false;
  if (  /*speed.Compute() && */ counting ) { // only sample when PID updates
    pos[p] = encoder0Pos;
    if (p < 999) p++;
    else counting = false;
  }
    pwmOut(output);
}


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
    case 'T': auto1 = !auto1; if(auto1) {aut++; aut%=3; Serial.println(aut);} break;
    case 'A': auto2 = !auto2; break;
    case 'Q': Serial.print("P="); Serial.print(kp); Serial.print(" I="); Serial.print(ki); Serial.print(" D="); Serial.print(kd); Serial.print(" VP="); Serial.print(vkp); Serial.print(" VI="); Serial.println(vki); break;
    case 'H': help(); break;
    case 'W': writetoEEPROM(); break;
    case 'K': eedump(); break;
    case 'R': recoverPIDfromEEPROM() ; break;
    case 'S': for (int i = 0; i < p; i++) Serial.println(pos[i]); break;
    case 'Z': detachInterrupt(2); break; // from then on, ignore step pulses (good for tests)
    case 'F': feed = Serial.parseFloat(); Serial.println(feed); break;
    case 'V': vkp = Serial.parseFloat(); speed.SetTunings(vkp, vki, vkd); break;
    case 'G': vki = Serial.parseFloat(); speed.SetTunings(vkp, vki, vkd); break;
    case 'Y': counting = true; for (int i = 0; i < p; i++) pos[i] = 0; p = 0; trapezoidal(Serial.parseInt()); break; // performs a trapezoidal move
    case '@': accel = Serial.parseFloat(); Serial.println(accel); break;
    case 'L': homing(); break;
    case 'B': counting = true; for (int i = 0; i < p; i++) pos[i] = 0; p = 0; senoidal(Serial.parseInt()); break; // performs a senoidal move
    case 'C': counting = true; for (int i = 0; i < p; i++) pos[i] = 0; p = 0; cosenoidal(Serial.parseInt()); break; // performs a cosenoidal move
    case '%': pwmOut(Serial.parseInt() ); int t=0; while(t<15) if(millis() % 1000 == 0) { t++; Serial.println(encoder0Pos); delay(1); } break; // I cannot return to position control

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
  Serial.println(F("L goes to home position and initalizes X0 coordinate"));
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
  long distance = destination - setpoint; // if positive go +x
  long finalPoint = setpoint + distance;
  boolean dirPos = true;
  if (distance < 0) {
    distance = -distance;  // me quedo con el valor absoluto del movimiento
    dirPos = false;
  }
  float xm = feed * feed / accel;
  float f=feed;
  float t1, t2;
    if (distance <= xm) { // triangular
      t1 = t2 = sqrt(distance / accel); 
      f = sqrt (accel * distance); // new max speed!!
  }
  else { // trapezoidal
    t1 = sqrt(xm / accel); // t1 = end of accel
    t2 = (distance - xm) / feed + t1; // t2 = end of coasting
  }
  Serial.print(t1); Serial.print(" "); Serial.println(t2);
  // Ok, I know what to do next, so let's perform the actual motion
  float t = 0, spd = 0.0;
  float dt = 1e-3;
  //float da = accel * dt;
  float covered = setpoint;
  float maxt = t1 + t2;
  while (t < maxt) {
    t += dt;
    if (t < t1) spd=t*f/t1; else if (t >= t2) spd=(maxt-t)*f/t1; 
    if ( dirPos ) covered += spd * dt; else covered -= spd * dt; // calculate new target position
    //vel =  encoder0Pos - input;
    input = encoder0Pos;
    setpoint = covered;
    while(!myPID.Compute()); // wait till PID is actually computer ('cause it was about time to).
    pwmOut(output );
    if (counting  ) {
      pos[p] = encoder0Pos;
      if (p < 999) p++;
      else counting = false;
    }
  }
  Serial.print(millis() - a1); Serial.print(F("  Err=")); Serial.println(encoder0Pos - covered);
  counting=false;
  target1 = covered;
}

//  if (t < t1) spd=(1-cos(3.14*t/t1))*feed/2; else if (t >= t2) spd=(1-cos(3.14*(maxt-t)/t1))*feed/2;
void senoidal(int destination) { // it will use acceleration and feed values to restrict the motion following a trapezoidal pattern
  long a1 = millis();
  long distance = destination - setpoint; // if positive go +x
  long finalPoint = setpoint + distance;
  boolean dirPos = true;
  if (distance < 0) {
    distance = -distance;  // me quedo con el valor absoluto del movimiento
    dirPos = false;
  }
  float xm = feed * feed / accel;
  float f=feed;
  float t1, t2;
    if (distance <= xm) { // triangular
      t1 = t2 = sqrt(distance / accel); 
      f = sqrt (accel * distance); // new max speed!!
  }
  else { // trapezoidal
    t1 = sqrt(xm / accel); // t1 = end of accel
    t2 = (distance - xm) / feed + t1; // t2 = end of coasting
  }
  Serial.print(t1); Serial.print(" "); Serial.println(t2);
  // Ok, I know what to do next, so let's perform the actual motion
  float t = 0, spd = 0.0;
  float dt = 1e-3;
  //float da = accel * dt;
  float covered = setpoint;
  float maxt = t1 + t2;
  long start=micros();
  while (t < maxt) {
    t=(micros()-start)/1e6; //t += dt;
    if (t < t1) spd=(1-cos(3.14*t/t1))*f/2; else if (t >= t2) spd=(1-cos(3.14*(maxt-t)/t1))*f/2;
    if ( dirPos ) covered += spd * dt; else covered -= spd * dt; // calculate new target position
    //vel =  encoder0Pos - input;
    input = encoder0Pos;
    setpoint = covered;
    while(!myPID.Compute()); // wait till PID is actually computer ('cause it was about time to).
    pwmOut(output );
    if (counting  ) {
      pos[p] = encoder0Pos;
      if (p < 999) p++;
      else counting = false;
    }
  }
  Serial.print(millis() - a1); Serial.print(F("  Err=")); Serial.println(encoder0Pos - covered);
  counting=false;
  target1 = covered;
}

void cosenoidal(int destination) { // it will use acceleration and feed values to restrict the motion following a trapezoidal pattern
  long a1 = millis();
  long distance = destination - setpoint; // if positive go +x
  long finalPoint = setpoint + distance;
  boolean dirPos = true;
  if (distance < 0) {
    distance = -distance;  // me quedo con el valor absoluto del movimiento
    dirPos = false;
  }
  float xm = feed * feed / accel;
  float tt,f=feed;
  float t1, t2;
    if (distance <= xm) { // triangular
      t1 = t2 = sqrt(distance / accel); 
      f = sqrt (accel * distance); // new max speed!!
  }
  else { // trapezoidal
    t1 = sqrt(xm / accel); // t1 = end of accel
    t2 = (distance - xm) / feed + t1; // t2 = end of coasting
  }
  Serial.print(t1); Serial.print(" "); Serial.println(t2);
  // Ok, I know what to do next, so let's perform the actual motion
  float t = 0, spd = 0.0;
  float dt = 1e-3;
  //float da = accel * dt;
  float covered = setpoint;
  float maxt = t1 + t2;
  long start=micros();
  while (t < maxt) {
    t=(micros()-start)/1e6; //t += dt;
    if (t < t1) {tt=6.28*t/t1; spd=f*(tt-sin(tt))/6.28;} else if (t >= t2) {tt=6.28*(maxt-t)/t1; spd=f*(tt-sin(tt))/6.28; }
    if ( dirPos ) covered += spd * dt; else covered -= spd * dt; // calculate new target position
    //vel =  encoder0Pos - input;
    input = encoder0Pos;
    setpoint = covered;
    while(!myPID.Compute()); // wait till PID is actually computed ('cause it was about time to).
    pwmOut( output );
    if (counting  ) {
      pos[p] = encoder0Pos;
      if (p < 999) p++;
      else counting = false;
    }
  }
  Serial.print(millis() - a1); Serial.print(F("  Err=")); Serial.println(encoder0Pos - covered);
  counting=false;
  target1 = covered;
}

// ------------------ feeding a varying position for trapezoidal motion 
 float t1,t2;
 float a=200,v=100;

float time(long L, float accel, float speed) { // 380 usec
  float spda = speed/accel;
  if(L<speed*spda) { // triangular
    //t1 = t2 = sqrt(L/accel);
    v = sqrt(a*L);  // max speed changes then
    t1 = t2 = v/a;
  }
  else {
    t1 = spda;
    t2 = L/speed;  
  }
  return t1+t2;
}

float et1=0, et2=0;

float position(long ms) { // reference position from zero to L (39 us)
  float t=ms/1000.0;
  float pos;
  if(t<t1) {  // accelerating ...
    pos = a*t*t/2; 
    et1=pos;
  }
  else if(t>=t1 && t<t2) { // coasting ...
    pos = et1 + v * (t-t1);
    et2 = pos; 
  }
  else if(t>=t2)  // decelerating
    pos = max(et1,et2) + v*(t-t2) - a*(t-t2)*(t-t2)/2;
  return pos;
}



