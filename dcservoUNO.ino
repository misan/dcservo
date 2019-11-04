
/* This one is not using any PinChangeInterrupt library */

/*
   This program uses an Arduino UNO for a closed-loop control of a DC-motor. 
   Motor motion is detected by a quadrature encoder.
   Two inputs named STEP and DIR allow changing the target position.
   Serial port prints current position and target position every second.
   Serial input can be used to feed a new location for the servo (no CR LF).
   
   Pins used:
   Digital inputs 2 & 8 are connected to the two encoder signals (AB).
   Digital input 3 is the STEP input.
   Analog input 0 is the DIR input.
   Digital input 4 is the HOME output. asserted if encoder0Pos is smaller than one.
   Digital outputs 9 & 10 control the PWM outputs for the motor (I am using half L298 here).


   Please note PID gains kp, ki, kd need to be tuned to each different setup. 
*/
#include <EEPROM.h>
#include <PID_v1.h>
#define encoder0PinA  2 // PD2; 
#define encoder0PinB  8  // PC0;
#define M1            9
#define M2            10  // motor's PWM outputs
#define ENDSTOP 4
#define STEP 3

byte pos[1000]; int p=0;

double kp=3,ki=0,kd=0.0;
double input=0, output=0, setpoint=0;
PID myPID(&input, &output, &setpoint,kp,ki,kd, DIRECT);
volatile long encoder0Pos = 0;
boolean isHome=true;
boolean auto1=false, auto2=false,counting=false;
long previousMillis = 0;        // will store last time LED was updated

long target1=0;  // destination location at any moment

//for motor control ramps 1.4
bool newStep = false;
bool oldStep = false;
bool dir = false;
byte skip=0;

// Install Pin change interrupt for a pin, can be called multiple times
void pciSetup(byte pin) 
{
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group 
}

void setup() { 
  pinMode(encoder0PinA, INPUT_PULLUP); 
  pinMode(encoder0PinB, INPUT_PULLUP);  
  pinMode(ENDSTOP, OUTPUT);  
  pinMode(STEP, INPUT_PULLUP);  
  pciSetup(encoder0PinB);
  attachInterrupt(0, encoderInt, CHANGE);  // encoder pin on interrupt 0 - pin 2
  attachInterrupt(1, countStep      , RISING);  // step  input on interrupt 1 - pin 3
  TCCR1B = TCCR1B & 0b11111000 | 1; // set 31Kh PWM
  Serial.begin (115200);
  help();
  recoverPIDfromEEPROM();
  //Setup the pid 
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);
  myPID.SetOutputLimits(-255,255);
  homing(); // comment if you don't want homming on reset
  help(); // display help 
} 

/*  enables to detect obstacles such as hard stops or soft rubber stops 
 *  without the use of a limit switch since it looks at the growing error 
 *  when such an event occurs.  
 *  an output pin is asserted after stop is detected for 2 seconds
 *  
 *  
 */
void homing(){
  long tstamp;    
  long error=0;
  int scanning_steps=20; // how fast you want to scan for home
  int max_error;
  float homing_power=1.00;  // power multiplicator for homing. suggested between 0.1 for 10% and 1 for 100%
  max_error=(scanning_steps*10+1);
  digitalWrite(ENDSTOP,0);  // Turn external pin low
  Serial.println("homing ...");
    while( error>-max_error){   // loop while error is less than max_error an obstacle or rubber stopper will make the error increase at each interval
      if(millis()-tstamp>7) // decreasetarget at desired time interval (6 default) 
        {
          setpoint-=scanning_steps;   //decrease target
          tstamp=millis();  //stamp the time
          Serial.print("setpoint ");          Serial.print(setpoint);
          Serial.print(" encoder ");          Serial.print(input);
          Serial.print(" error ");          Serial.println(error);
        }
    input = encoder0Pos;
    error=setpoint-input;  
    while(!myPID.Compute()); // wait till PID is actually computed
    pwmOut(output*homing_power); 
  }
  encoder0Pos=-50; // detected limit is now -70 to (if a soft limit is set like rubber motor would always try to push if zer0)
  target1=0; // target is now the new zero 
}


void loop(){
    input = encoder0Pos; 
    setpoint=target1;
    while(!myPID.Compute())
           {endstop();} // wait till PID is actually computed in the mean time assert the endstop pins
    if(Serial.available()) process_line(); // it may induce a glitch to move motion, so use it sparingly 
    if(input==setpoint)pwmOut(0); else pwmOut(output); 
    if(auto1) if(millis() % 1000 == 0) target1=random(9000); // that was for self test with no input from main controller
    if(auto2) if(millis() % 1000 == 0) printPos();
    //if(counting && abs(input-target1)<15) counting=false; 
    if(counting &&  (skip++ % 5)==0 ) {pos[p]=encoder0Pos; if(p<999) p++; else counting=false;}
    endstop(); // output status of endstop
}

void pwmOut(int out) {
   if(out<0) { analogWrite(M1,0); analogWrite(M2,abs(out)); }
   else { analogWrite(M2,0); analogWrite(M1,abs(out)); }
  }

const int QEM [16] = {0,-1,1,2,1,0,2,-1,-1,2,0,1,2,1,-1,0};               // Quadrature Encoder Matrix
static unsigned char New, Old;
ISR (PCINT0_vect) { // handle pin change interrupt for D8
  Old = New << 2;
  New = (PINB & 1 )+ ((PIND & 4) >> 1); //
  encoder0Pos+= QEM [Old + New];

}

void encoderInt() { // handle pin change interrupt for D2
  Old = New << 2;
  New = (PINB & 1 )+ ((PIND & 4) >> 1); //
  encoder0Pos+= QEM [Old + New];

}


void countStep(){ if (PINC&B0000001) target1--;else target1++; } // pin A0 represents direction

void process_line() {
 char cmd = Serial.read();
 if(cmd>'Z') cmd-=32;
 switch(cmd) {
  case 'P': kp=Serial.parseFloat(); myPID.SetTunings(kp,ki,kd); break;
  case 'D': kd=Serial.parseFloat(); myPID.SetTunings(kp,ki,kd); break;
  case 'I': ki=Serial.parseFloat(); myPID.SetTunings(kp,ki,kd); break;
  case '?': printPos(); break;
  case 'X': target1=Serial.parseInt(); p=0; counting=true; for(int i=0; i<300; i++) pos[i]=0; break;
  case 'T': auto1 = !auto1; break;
  case 'A': auto2 = !auto2; break;
  case 'Q': Serial.print("P="); Serial.print(kp); Serial.print(" I="); Serial.print(ki); Serial.print(" D="); Serial.println(kd); break;
  case 'H': help(); break;
  case 'W': writetoEEPROM(); break;
  case 'K': eedump(); break;
  case 'R': recoverPIDfromEEPROM() ; break;
  case 'S': for(int i=0; i<p; i++) Serial.println(pos[i]); break;
  case 'L': homing(); break;
 }
 while(Serial.read()!=10); // dump extra characters till LF is seen (you can use CRLF or just LF)
}

void printPos() {
  Serial.print(F("Position=")); Serial.print(encoder0Pos); Serial.print(F(" PID_output=")); 
  Serial.print(output); Serial.print(F(" Target=")); Serial.print(setpoint);
  if(!isHome)  Serial.print(" NOT");  Serial.println(" home");
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
 Serial.println(F("L will execute homing\n")); 
}

void writetoEEPROM() { // keep PID set values in EEPROM so they are kept when arduino goes off
  eeput(kp,0);
  eeput(ki,4);
  eeput(kd,8);
  double cks=0;
  for(int i=0; i<12; i++) cks+=EEPROM.read(i);
  eeput(cks,12);
  Serial.println("\nPID values stored to EEPROM");
  //Serial.println(cks);
}

void recoverPIDfromEEPROM() {
  double cks=0;
  double cksEE;
  for(int i=0; i<12; i++) cks+=EEPROM.read(i);
  cksEE=eeget(12);
  //Serial.println(cks);
  if(cks==cksEE) {
    Serial.println(F("*** Found PID values on EEPROM"));
    kp=eeget(0);
    ki=eeget(4);
    kd=eeget(8);
    myPID.SetTunings(kp,ki,kd); 
  }
  else Serial.println(F("*** Bad checksum"));
}

void eeput(double value, int dir) { // Snow Leopard keeps me grounded to 1.0.6 Arduino, so I have to do this :-(
  char * addr = (char * ) &value;
  for(int i=dir; i<dir+4; i++)  EEPROM.write(i,addr[i-dir]);
}

double eeget(int dir) { // Snow Leopard keeps me grounded to 1.0.6 Arduino, so I have to do this :-(
  double value;
  char * addr = (char * ) &value;
  for(int i=dir; i<dir+4; i++) addr[i-dir]=EEPROM.read(i);
  return value;
}

void eedump() {
 for(int i=0; i<16; i++) { Serial.print(EEPROM.read(i),HEX); Serial.print(" "); }Serial.println(); 
}

void endstop (){
   // endstop detection.  it is interlocked.  under normal operation it would not
  // send a digital write, just on transition smaller than 2;
  // this would not significantly affect normal operation. 
  if(encoder0Pos<2)
    {
      if(encoder0Pos<=0&&!isHome)  
      {
        isHome=true; 
        digitalWrite(ENDSTOP,isHome);
      }
  else if(encoder0Pos>0&&isHome)
      {
        isHome=false; 
        digitalWrite(ENDSTOP,isHome);
      }
    }
}
