// Code for MakeIt Venty
// by Nick Serpa, Jonathan Vail, Phil Martel
// License ??
/* Brief description:
 *  Code for an Arduino driven AMBU_based ventilator
 *  based on the MIT e-vent https://e-vent.mit.edu/
 */
 /* TO-DO:
  *  1 - Button board fix - One increment/decrement of BPM, Tidal, and I/E per button push (Phil Completed)
  *  2 - Pressure sensor code - Set off a high or low alarm, will likely output to LED
  *  3 - Initialization of the arm by having it contact the upper limit switch, setting position to zero, and waiting for sin function to match before  enabling PID driven motion
  *  4 - Sin wave function optimization - Create the data for each BPM and Tidal volume option and integrate
  *  5 - Limit the tidal and BPM values as well as scroll through available options for I:E (Phil completed)
  *  6 - Analog gauge for -1.5psig to +1.5psig for calibrating / referencing
  *  7 - License info needed and discolusre 
  */

#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <Encoder.h>
#include <Streaming.h> // this supporting the << streaming operator, which allows more compact Serial.print
//#include "venty-table.h"
//#include <Button.h>  // button class (Button-Arduino library) that will enable greater capabilities in the future

boolean toggle4 = LOW;
unsigned long tick = 0;
unsigned long mtime;
const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to
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

//Button variable definitions
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
//float sinVal[24] = {1.0000, 0.9829, 0.9330, 0.8536, 0.7500, 0.6294, 0.5000, 0.3706, 0.2500, 0.1464, 0.0670, 0.0170, 0.0000, 0.0170, 0.0670, 0.1464, 0.2500, 0.3706, 0.5000, 0.6294, 0.7500, 0.8536, 0.9330, 0.9830};
float ScaledSinVal = 0.0;
int sinId = 0;
long millisComp = 250;
float SinScale = 712.0;

#define ENCODER_A 2
#define ENCODER_B 3
#define MOTORPIN 11

LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 20 x 4 line display

Encoder myEnc(ENCODER_A, ENCODER_B);

//Button process initiialization  
// make sure x is between L and H
#define MAKE_BETWEEN(L,x,H)  x = max((L),min((H),x))

/* limit aand increment information from The MakeIt Labs Vent Overfies document and its links
 *  https://docs.google.com/document/d/1q4NgTeFqrK2djGSt-CwOr3A_8AN943n714Y1b6zSWf0/edit
 *  
 *  Variable    min   max   step
 *  bpm         8     40    2
 *  tidalVol    200   800   50
 *  ieratio     1     4     1       Note I/E ratio is expressed as 1:N just work with N here
 */

#define BPM_MIN   8
#define BPM_MAX   40
#define BPM_STEP  2
#define TIDAL_MIN   200
#define TIDAL_MAX   800
#define TIDAL_STEP  50
#define IE_MIN   1
#define IE_MAX   4
#define IE_STEP  1

// variables that depend on BPM and I/E ratio  These should be set when the parameters change
float msecPerBreath, inhaleMsec, exhaleMsec;

void setup()
{
  
//Initialize buttons
    buttonInit();

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

//Button function call
  buttonProcess();

//case output function call
//  caseFunc();

//sin output function call
  sinfunc01();

//PID loop function call
  PIDloop();

//LCD layout original funtion call
//  VentyLcdOld();

//New LCD layout function call
  VentyLcdNew();

  // read the analog in value:
  sensorValue = analogRead(analogInPin);
  // map it to the range of the analog out:
  outputValue = map(sensorValue, 0, 1023, 64, 128);
  // change the analog out value:
  //analogWrite(analogOutPin, outputValue);

//  Serial.print("sensor = ");
//  Serial.print(sensorValue);
//  Serial.print("\t output = ");
//  Serial.print(outputValue);

//  Serial.print("\t Speed = ");
//  Serial.print(speed);
//  Serial.print("\t Case = ");
//  Serial.print(caseNum);
//  Serial.print("\t Tick = ");
//  Serial.print(tick);
//  Serial.print("\t BPM = ");
//  Serial.print(bpm);
//  Serial.print("\t IE Ratio = ");
//  Serial.print(ieratio);
//  Serial.print("\t Tidal Vol = ");
//  Serial.print(tidalvol);
//  Serial.print("\t vPin = ");
//  Serial.print(vstate);
//  Serial.print("\t uPin = ");
//  Serial.print(ustate);
//  Serial.print("\t tPin = ");
//  Serial.print(tstate);
//  Serial.print("\t sPin = ");
//  Serial.print(sstate);
//  Serial.print("\t nPin = ");
//  Serial.print(nstate);
//  Serial.print("\t mPin = ");
//  Serial.print(mstate);
//  Serial.print("\t lPin = ");
//  Serial.print(lstate);
//  Serial.print("\t kPin = ");
//  Serial.print(kstate);
//  Serial.print("\t Time = ");
//  Serial.print(mtime);

//Serial << "I/E ratio = 1:" << ieratio << " bpm = " << bpm << " tidal = " << tidalvol << endl; 

//  Serial.print("millisComp = ");
//  Serial.print(millisComp);
//  Serial.print("\t sinId = ");
//  Serial.print(sinId);
//  Serial.print("\t sinVal = ");
//  Serial.print(sinVal[sinId]);
//  Serial.print("\t ScaledSin = ");
//  Serial.print(ScaledSinVal);
//  Serial.print("\t PIDError = ");
//  Serial.print(PIDError);
//  Serial.print("\t PID_Value = ");
//  Serial.print(PID_Value);
//  Serial.print("\t P = ");
//  Serial.print(PID_p);
//  Serial.print("\t I = ");
//  Serial.print(PID_i);
//  Serial.print("\t D = ");
//  Serial.print(PID_d);
//  Serial.print("\t EncNewPos = ");
//  Serial.print(newPosition);
//  Serial.print("\t PID Out = ");
//  Serial.print(PID_out);
//  Serial.print("\t SWstate = ");
//  Serial.print(swstate);
  
//  Serial.println();

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
  //ScaledSinVal = sinVal[sinId] * SinScale;
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


//OLD LCD layout
void VentyLcdOld(){
    lcd.setCursor(0,0);
    lcd.print("Venty");
    lcd.setCursor(7,0);
    lcd.print("C=");
    lcd.print(caseNum);
    lcd.print(" ");
    lcd.setCursor(11,0);
    lcd.print("S=");
    lcd.print(speed);
    lcd.print(" ");
    lcd.setCursor(0,1);
    lcd.print("Pot=");    
    lcd.print(sensorValue);
    lcd.print(" ");
    lcd.setCursor(8,1);
    lcd.print("Enc=");    
    lcd.print(newPosition);
    lcd.print("   ");
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


// button code
void buttonInit(void) {
    pinMode(kpin, INPUT_PULLUP);
    pinMode(lpin, INPUT_PULLUP);
    pinMode(mpin, INPUT_PULLUP);
    pinMode(npin, INPUT_PULLUP);
    pinMode(spin, INPUT_PULLUP);
    pinMode(tpin, INPUT_PULLUP);
    pinMode(vpin, INPUT_PULLUP);
    pinMode(swpin, INPUT_PULLUP);
}

void buttonProcess(void) {
    // simple "inc/dec once per push changes" -Phil
    // rearranging so the inc and dec are together -Phil
    // the if condition will be true when the button is newly pushed =Phil

    if((kstate == HIGH) && (digitalRead(kpin) == LOW)){tidalvol -= TIDAL_STEP;} //10
    kstate=digitalRead(kpin);
    // same thing, but a little more compact -Phil
    if(sstate && !digitalRead(spin)){tidalvol += TIDAL_STEP;}  //5
    sstate=digitalRead(spin);
    MAKE_BETWEEN( TIDAL_MIN, tidalvol, TIDAL_MAX );

    if(tstate && !digitalRead(tpin)){ieratio += IE_STEP;} // 4
    tstate=digitalRead(tpin);
    if(lstate && !digitalRead(lpin)){ieratio -= IE_STEP;} //9
    lstate=digitalRead(lpin);
    MAKE_BETWEEN( IE_MIN, ieratio, IE_MAX );

    if(vstate && !digitalRead(vpin)){bpm += BPM_STEP;} //12
    vstate=digitalRead(vpin);
    if(nstate && !digitalRead(npin)){bpm -= BPM_STEP;} //6
    nstate=digitalRead(npin);
    MAKE_BETWEEN( BPM_MIN, bpm, BPM_MAX );

    // update parameters that depend on inputs
    updateParameters();
}

// update parameters that depend on inputs
// we may want to only do this during certain states, sau at the end of a breath
void updateParameters(void) {
  // static variables retain the value between calls to the function
  static int oldBPM = 0;
  static int oldTidalVol = 0;
  static int oldIERatio = 0;

  if ( (oldBPM != bpm) || (oldIERatio != ieratio) || (oldTidalVol != tidalvol) ) {
    //update static variables
    oldBPM = bpm;
    oldTidalVol = tidalvol;
    oldIERatio = ieratio;
    // update changed parameters
    msecPerBreath = 60000./bpm;
    inhaleMsec = msecPerBreath/(ieratio +1);
    exhaleMsec = inhaleMsec * ieratio;
    Serial << "updated params BPM:" << bpm << " ieratio: 1:"<< ieratio << " tidal volume:" << tidalvol << endl;
    Serial << "msec/breath:" << msecPerBreath << " inhale:" << inhaleMsec << " exhale:" << exhaleMsec << endl;
  }

}
