// Code for MakeIt Venty
// by Nick Serpa, Jonathan Vail, Phil Martel
// License ??
/* Bried description:
 *  Code for an Arduino driven AMBU_based ventilator
 *  based on the MIT e-vent https://e-vent.mit.edu/
 */
 /* TO-DO:
  *  1 - Button board fix - One increment/decrement of BPM, Tidal, and I/E per button push (Phil Completed)
  *  2 - Pressure sensor code - Set off a high or low alarm, will likely output to LED
  *  3 - Initialization of the arm by having it contact the upper limit switch, setting position to zero, and waiting for sin function to match before  enabling PID driven motion
  *  4 - Sin wave function optimization - Create the data for each BPM and Tidal volume option and integrate
  *  5 - Limit the tidal and BPM values as well as scroll through available options for I:E
  */
  
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <Encoder.h>

#include <Button.h>  // button class that will enable greater capabilities

// Working on limits for tidal, I/E, BPM - Phil
boolean toggle4 = LOW;
unsigned long tick = 0;
unsigned long mtime;
const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to
// not used.  It's called MOTORPIN.  const int analogOutPin = 11; // Analog output pin that the LED is attached to
int sensorValue = 0;        // value read from the pot
int outputValue = 0;        // value output to the PWM (analog out)
int speed;
int prestate = 0;
int caseNum;
long oldPosition  = -999;
long newPosition;
long state = 0;
int bpm = 10;
int tidalvol = 400;
int ieratio =1;
// making the pins const int
boolean start = LOW;
const int kpin = 10;
boolean kstate = 0;
const int lpin = 9;
boolean lstate = 0;
const int mpin = 8;
boolean mstate = 0;
const int npin = 6;
boolean nstate = 0;
const int spin = 5;
boolean sstate = 0;
const int tpin = 4;
boolean tstate = 0;
//int upin = 13;
//boolean ustate = 0;
const int vpin = 12;
boolean vstate = 0;
const int swpin = 7;
boolean swstate = 0;
//PID variable definition
float kp = 1.0;
float ki = 0.0;
float kd = 0.0;
int PID_p = 0;
int PID_i = 0;
int PID_d = 0;
float last_kp = 0.0;
float last_ki = 0.0;
float last_kd = 0.0;
float PID_Value = 0.0;
int PID_values_fixed = 0;
float PIDError = 0;
float previous_error = 0.0;
float PreviousError = 0.0;
float ElapsedTime = 1.0;
float Time;
float TimePrev;
int PID_out = 0;
//Sin wave variable definition
float sinVal[24] = {1.0000, 0.9829, 0.9330, 0.8536, 0.7500, 0.6294, 0.5000, 0.3706, 0.2500, 0.1464, 0.0670, 0.0170, 0.0000, 0.0170, 0.0670, 0.1464, 0.2500, 0.3706, 0.5000, 0.6294, 0.7500, 0.8536, 0.9330, 0.9830};
float ScaledSinVal = 0.0;
int sinId = 0;
long millisComp = 250;
float SinScale = 712.0;
#define ENCODER_A 2
#define ENCODER_B 3
#define MOTORPIN 11
LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 20 x 4 line display
Encoder myEnc(ENCODER_A, ENCODER_B);
void setup()
{
//Interface pin config
pinMode(kpin, INPUT_PULLUP);
pinMode(lpin, INPUT_PULLUP);
pinMode(mpin, INPUT_PULLUP);
pinMode(npin, INPUT_PULLUP);
pinMode(spin, INPUT_PULLUP);
pinMode(tpin, INPUT_PULLUP);
pinMode(vpin, INPUT_PULLUP);
pinMode(swpin, INPUT_PULLUP);
pinMode(A3, INPUT); // analog
pinMode(LED_BUILTIN, OUTPUT);
pinMode(MOTORPIN, OUTPUT);
//TIMER2 SETUP
TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
TCCR2B = _BV(CS22)|_BV(CS21)  ;
Serial.begin(115200);
Serial.println("Venty Startup");
analogReference(DEFAULT);
// initialize the lcd 
  lcd.init();
  lcd.backlight();
//lcd.noBacklight();
}
void loop()
{
    //encoder position set
    newPosition = myEnc.read();
    long encspeed = newPosition-oldPosition;
    mtime = millis();
    //set old position for next scan 
    oldPosition = newPosition;
//interface pins read and states
    swstate=digitalRead(swpin); // just printed right now -Phil
    mstate=digitalRead(mpin);  // unused right now -Phil
 
    // simple "inc/dec once per push changes" -Phil
    // rearranging so the inc and dec are together -Phil
    // the if condition will be true when the button is newly pushed =Phil
    if((kstate == HIGH) && (digitalRead(kpin) == LOW)){tidalvol --;}
    kstate=digitalRead(kpin);
    // same thing, but a little more compact -Phil
    if(sstate && !digitalRead(spin)){tidalvol ++;}
    sstate=digitalRead(spin);
    
    if(tstate && !digitalRead(tpin)){ieratio ++;}
    tstate=digitalRead(tpin);
    if(lstate && !digitalRead(lpin)){ieratio --;}
    lstate=digitalRead(lpin);
    
    if(vstate && !digitalRead(vpin)){bpm ++;}
    vstate=digitalRead(vpin);
    if(nstate && !digitalRead(npin)){bpm --;}
    nstate=digitalRead(npin);
 
//case output function call
//  caseFunc();
//sin output function call
  sinfunc01();
//PID loop function call
  PIDloop();
//New LCD layout function call
  VentyLcdNew();
  // read the analog in value:
  sensorValue = analogRead(analogInPin);
  // map it to the range of the analog out:
  outputValue = map(sensorValue, 0, 1023, 64, 128);
  // change the analog out value:
  //analogWrite(analogOutPin, outputValue);
  Serial.print("millisComp = ");
  Serial.print(millisComp);
  Serial.print("\t sinId = ");
  Serial.print(sinId);
  Serial.print("\t sinVal = ");
  Serial.print(sinVal[sinId]);
  Serial.print("\t ScaledSin = ");
  Serial.print(ScaledSinVal);
  Serial.print("\t PIDError = ");
  Serial.print(PIDError);
  Serial.print("\t PID_Value = ");
  Serial.print(PID_Value);
  Serial.print("\t P = ");
  Serial.print(PID_p);
  Serial.print("\t I = ");
  Serial.print(PID_i);
  Serial.print("\t D = ");
  Serial.print(PID_d);
  Serial.print("\t EncNewPos = ");
  Serial.print(newPosition);
  Serial.print("\t PID Out = ");
  Serial.print(PID_out);
  Serial.print("\t SWstate = ");
  Serial.print(swstate);
  
  Serial.println();
  tick++;
  if (tick%50 == 0) {
    
//    lcd.clear(); //Clears the LCD of previous writes
//    lcd.setCursor(0,0);  
//    lcd.print("Venty  C=  S=   ");
//    lcd.setCursor(0,1);    
//    lcd.print("Pot=    Enc=    ");
  }
//case controlled output
// analogWrite(MOTORPIN,speed); 
//potentiometer controlled output
//analogWrite(MOTORPIN,outputValue);
//PID controlled output
  analogWrite(MOTORPIN,PID_out);
  
  delay(5);
  digitalWrite( LED_BUILTIN, !digitalRead(LED_BUILTIN));  //Toggle 1/loop for timing
}
//Sine wave stage funtion 
void sinfunc01 (){
  if (millis() < millisComp){
    
  }
  if (millis() >= millisComp){
    sinId++;
    millisComp = millisComp + 250;
  }
  if (sinId >= 23){
    sinId = 0;
  }
  ScaledSinVal = sinVal[sinId] * SinScale;
}
//PID loop function
void PIDloop(){
  TimePrev = Time;
  Time = millis();
  PIDError = ScaledSinVal - newPosition;
  PID_p = kp * PIDError;
  PID_i = PID_i + (ki * PIDError);
  PID_d = kd * ((PIDError - previous_error)/ElapsedTime);
  PID_Value = PID_p + PID_i + PID_d;
  PID_out = map(PID_Value, 0, SinScale, 64, 128);
}
//New LCD layout
void VentyLcdNew(){
    lcd.setCursor(0,0);  
    lcd.print("BPM");
    lcd.setCursor(5,0);
    lcd.print("TIDAL");
    lcd.setCursor(13,0);
    lcd.print("I/E");
    lcd.setCursor(1,1);
    lcd.print(bpm);
    lcd.print("  ");
    lcd.setCursor(6,1);
    lcd.print(tidalvol);
    lcd.print("  ");
    lcd.setCursor(13,1);
    lcd.print("1:");
    lcd.print(ieratio);
}
//Case based output function
void caseFunc(){
 if (++prestate >= 2) {
    prestate = 0;
 // temporary comment digitalWrite(LED_BUILTIN,toggle4);
 toggle4 = !toggle4;
  state++;
  switch ((state/256) % 8) {
  case 0:
    speed = 64+ ((state%256)/4);
//      speed = 64;
      caseNum = 0;
  break;
  
  case 1:
      caseNum = 1;
  break;
  
  case 2:
    speed = (255-(state%256))/4;
//     speed = 96;
      caseNum = 2;
  break;
  
  case 3:
      caseNum = 3;
  break;
  
  case 4:
    speed = 64+ (state%256)/4;
//      speed = 128;
      caseNum = 4;
  break;
  
  case 5:
      caseNum = 5;
  break;
  
  case 6:
    speed = 64+(255-(state%256))/4;
//      speed = 96;
      caseNum = 6;
  break;
  
  case 7:  
      caseNum = 7;
  break;
  }
 }
}
