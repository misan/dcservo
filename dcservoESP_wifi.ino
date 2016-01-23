/*
 * Miguel Sanchez 2106
   Mauro Manco 2016 Porting on ESP8266
   
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

#include <EEPROM.h>
#include <PID_v1.h>
#include <ESP8266WiFi.h>

const char* ssid = "*****";
const char* password = "****";

// Create an instance of the server
// specify the port to listen on as an argument
WiFiServer server(23);
WiFiClient client;

const int encoder0PinA = 13;
const int encoder0PinB = 12;
const int Step = 14;
const int M1=16;
const int M2=5;
const int DIR=4;
const int PWM_MOT=15;

byte pos[1000]; int p=0;
double kp=3,ki=0,kd=0.0;
double input=0, output=0, setpoint=0;
PID myPID(&input, &output, &setpoint,kp,ki,kd, DIRECT);
volatile long encoder0Pos = 0;
boolean auto1=false, auto2=false,counting=false;
long previousMillis = 0;        // will store last time LED was updated

long target1=0;  // destination location at any moment

//for motor control ramps 1.4
bool newStep = false;
bool oldStep = false;
bool dir = false;
byte skip=0;

void toggle() {
  static int state = 0;
  state = !state;
  digitalWrite(BUILTIN_LED, state);
}



  void pwmOut(int out) {
   if(out>0) { digitalWrite(M1,0); analogWrite(M2,out); }
   else      { analogWrite(M1,-out); digitalWrite(M2,0); }
   //analogWrite(PWM_MOT,abs(out));
   //PWM = out;
  }

const int QEM [16] = {0,-1,1,2,1,0,2,-1,-1,2,0,1,2,1,-1,0};               // Quadrature Encoder Matrix
static unsigned char New, Old;


void encoderInt() { // handle pin change interrupt for D2
  Old = New;
  //New = PIND & 3; //(PINB & 1 )+ ((PIND & 4) >> 1); //   Mauro Manco
  New = digitalRead(encoder0PinA)*2 + digitalRead(encoder0PinB);
  encoder0Pos+= QEM [Old * 4 + New];
}


void countStep(){ if (digitalRead(DIR)== HIGH) target1--;else target1++;
} // pin A0 represents direction == PF7 en Pro Micro



void printPos() {
  client.print(F("Position=")); client.print(encoder0Pos); client.print(F(" PID_output=")); client.print(output); client.print(F(" Target=")); client.println(setpoint);
}



void help() {
 client.println(F("\nPID DC motor controller and stepper interface emulator"));
 client.println(F("by misan - porting cured by Exilaus"));
 client.println(F("Available serial commands: (lines end with CRLF or LF)"));
 client.println(F("P123.34 sets proportional term to 123.34"));
 client.println(F("I123.34 sets integral term to 123.34"));
 client.println(F("D123.34 sets derivative term to 123.34"));
 client.println(F("? prints out current encoder, output and setpoint values"));
 client.println(F("X123 sets the target destination for the motor to 123 encoder pulses"));
 client.println(F("T starts a sequence of random destinations (between 0 and 2000) every 3 seconds. T again will disable that"));
 client.println(F("Q prints out the current values of P, I and D parameters")); 
 client.println(F("W stores current values of P, I and D parameters into EEPROM")); 
 client.println(F("H prints this help message again")); 
 client.println(F("A toggles on/off showing regulator status every second\n")); 
 client.println(F("B closes the connection\n")); 
}


void eeput(double value, int dir) { // Snow Leopard keeps me grounded to 1.0.6 Arduino, so I have to do this :-(
  char * addr = (char * ) &value;
  for(int i=dir; i<dir+4; i++)  EEPROM.write(i,addr[i-dir]);
}


void writetoEEPROM() { // keep PID set values in EEPROM so they are kept when arduino goes off
  eeput(kp,0);
  eeput(ki,4);
  eeput(kd,8);
  double cks=0;
  for(int i=0; i<12; i++) cks+=EEPROM.read(i);
  eeput(cks,12);
  client.println("\nPID values stored to EEPROM");
  //Serial.println(cks);
}


double eeget(int dir) { // Snow Leopard keeps me grounded to 1.0.6 Arduino, so I have to do this :-(
  double value;
  char * addr = (char * ) &value;
  for(int i=dir; i<dir+4; i++) addr[i-dir]=EEPROM.read(i);
  return value;
}

void recoverPIDfromEEPROM() {
  double cks=0;
  double cksEE;
  for(int i=0; i<12; i++) cks+=EEPROM.read(i);
  cksEE=eeget(12);
  //Serial.println(cks);
  if(cks==cksEE) {
    client.println(F("*** Found PID values on EEPROM"));
    kp=eeget(0);
    ki=eeget(4);
    kd=eeget(8);
    myPID.SetTunings(kp,ki,kd); 
  }
  else client.println(F("*** Bad checksum"));
}


void eedump() {
 for(int i=0; i<16; i++) { client.print(EEPROM.read(i),HEX); client.print(" "); }client.println(); 
}


void setup() {
  Serial.begin (115200);
  pinMode(BUILTIN_LED, OUTPUT);
  pinMode(encoder0PinA, INPUT);
  pinMode(encoder0PinB, INPUT);
  pinMode(Step, INPUT);
  pinMode(PWM_MOT, OUTPUT);
  pinMode(M1,OUTPUT);
  pinMode(M2,OUTPUT);
  analogWriteFreq(20000);  // set PWM to 20Khz
  analogWriteRange(255);   // set PWM to 255 levels (not sure if more is better)
  attachInterrupt(encoder0PinA, encoderInt, CHANGE);
  attachInterrupt(encoder0PinB, encoderInt, CHANGE);
  attachInterrupt(Step, countStep, RISING);
  toggle();
  help();
  recoverPIDfromEEPROM();
  //Setup the pid 
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);
  myPID.SetOutputLimits(-255,255);

    // Connect to WiFi network
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  
  // Start the server
  server.begin();
  Serial.println("Server started");

  // Print the IP address
  Serial.println(WiFi.localIP());
}

void process_line() {
 char cmd = client.read();
 if(cmd>'Z') cmd-=32;
 switch(cmd) {
  case 'P': kp=client.parseFloat(); myPID.SetTunings(kp,ki,kd); break;
  case 'D': kd=client.parseFloat(); myPID.SetTunings(kp,ki,kd); break;
  case 'I': ki=client.parseFloat(); myPID.SetTunings(kp,ki,kd); break;
  case '?': printPos(); break;
  case 'X': target1=client.parseInt(); counting=true; for(int i=0; i<p; i++) pos[i]=0; p=0; break;
  case 'T': auto1 = !auto1; break;
  case 'A': auto2 = !auto2; break;
  case 'Q': client.print("P="); client.print(kp); client.print(" I="); client.print(ki); client.print(" D="); client.println(kd); break;
  case 'H': help(); break;
  case 'W': writetoEEPROM(); break;
  case 'K': eedump(); break;
  case 'R': recoverPIDfromEEPROM() ; break;
  case 'S': for(int i=0; i<p; i++) client.println(pos[i]); break;
  case 'B': client.stop(); break;
 }
// while(Serial.read()!=10); // dump extra characters till LF is seen (you can use CRLF or just LF)
}



void loop() {
    if(!client) client = server.available();
    input = encoder0Pos; 
    setpoint=target1;
    myPID.Compute();
    //if(Serial.available()) process_line();
    if(client && client.available()) process_line();
    pwmOut(output); 
    if(auto1) if(millis() % 3000 == 0) target1=random(2000); // that was for self test with no input from main controller
    if(auto2) if(millis() % 1000 == 0) printPos();
    if(counting && abs(input-target1)<15) counting=false; 
    if(counting &&  (skip++ % 5)==0 ) {pos[p]=encoder0Pos; if(p<999) p++; else counting=false;}

   }
