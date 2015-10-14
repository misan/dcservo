
/* This one is not using any PinChangeInterrupt library */

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


double kp=3,ki=0,kd=0.0;
double input=80, output=0, setpoint=180;
PID myPID(&input, &output, &setpoint,kp,ki,kd, DIRECT);
volatile long encoder0Pos = 0;
boolean auto1=false;
long previousMillis = 0;        // will store last time LED was updated

long target1=0;  // destination location at any moment

//for motor control ramps 1.4
bool newStep = false;
bool oldStep = false;
bool dir = false;

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
  attachInterrupt(1, countStep      , RISING);  // step  input on interrupt 1 - pin 3
  TCCR1B = TCCR1B & 0b11111000 | 1; // set 31Kh PWM
  Serial.begin (115200);
  help();
  recoverPIDfromEEPROM();
  //Setup the pid 
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);
  myPID.SetOutputLimits(-255,255);
} 

void loop(){
    input = encoder0Pos; 
    setpoint=target1;
    myPID.Compute();
    // interpret received data as an integer (no CR LR): provision for manual testing over the serial port
    if(Serial.available()) process_line(); // it may induce a glitch to move motion, so use it sparingly 
    pwmOut(output); 
  if(auto1) if(millis() % 3000 == 0) target1=random(2000); // that was for self test with no input from main controller
  //if(millis()%1000 == 0) Serial.println(encoder0Pos);
}

void pwmOut(int out) {
   if(out<0) { analogWrite(M1,0); analogWrite(M2,abs(out)); }
   else { analogWrite(M2,0); analogWrite(M1,abs(out)); }
  }

const int QEM [16] = {0,-1,1,2,1,0,2,-1,-1,2,0,1,2,1,-1,0};               // Quadrature Encoder Matrix
static unsigned char New, Old;
ISR (PCINT0_vect) { // handle pin change interrupt for D8
  Old = New;
  New = (PINB & 1 )+ ((PIND & 4) >> 1); //
  encoder0Pos+= QEM [Old * 4 + New];
}

void encoderInt() { // handle pin change interrupt for D8
  Old = New;
  New = (PINB & 1 )+ ((PIND & 4) >> 1); //
  encoder0Pos+= QEM [Old * 4 + New];
}


void countStep(){ if (PINC&B0000001) target1--;else target1++; } // pin A0 represents direction

void process_line() {
 char cmd = Serial.read();
 if(cmd>'Z') cmd-=32;
 switch(cmd) {
  case 'P': kp=Serial.parseFloat(); myPID.SetTunings(kp,ki,kd); break;
  case 'D': kd=Serial.parseFloat(); myPID.SetTunings(kp,ki,kd); break;
  case 'I': ki=Serial.parseFloat(); myPID.SetTunings(kp,ki,kd); break;
  case '?': Serial.print(encoder0Pos); Serial.print(" "); Serial.print(output); Serial.print(" "); Serial.println(setpoint); break;
  case 'X': target1=Serial.parseInt(); break;
  case 'T': auto1 = !auto1; break;
  case 'Q': Serial.print("P="); Serial.print(kp); Serial.print(" I="); Serial.print(ki); Serial.print(" D="); Serial.println(kd); break;
  case 'H': help(); break;
  case 'W': writetoEEPROM(); break;
  case 'K': eedump(); break;
  case 'R': recoverPIDfromEEPROM() ; break;
 }
 while(Serial.read()!=10); // dump extra characters till LF is seen (you can use CRLF or just LF)
}

void help() {
 Serial.println("\nPID DC motor controller and stepper interface emulator");
 Serial.println("by misan");
 Serial.println("Available serial commands: (lines end with CRLF or LF)");
 Serial.println("P123.34 sets proportional term to 123.34");
 Serial.println("I123.34 sets integral term to 123.34");
 Serial.println("D123.34 sets derivative term to 123.34");
 Serial.println("? prints out current encoder, output and setpoint values");
 Serial.println("X123 sets the target destination for the motor to 123 encoder pulses");
 Serial.println("T will start a sequence of random destinations (between 0 and 2000) every 3 seconds. T again will disable that");
 Serial.println("Q will print out the current values of P, I and D parameters"); 
 Serial.println("W will store current values of P, I and D parameters into EEPROM"); 
 Serial.println("H will print this help message again\n"); 
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
    Serial.println("*** Found PID values on EEPROM");
    kp=eeget(0);
    ki=eeget(4);
    kd=eeget(8);
    myPID.SetTunings(kp,ki,kd); 
  }
  else Serial.println("*** Bad checksum");
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
