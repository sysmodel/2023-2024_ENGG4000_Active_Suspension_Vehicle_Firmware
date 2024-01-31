/* 
* Rack and Pinion Test Jig Firmware (w/ current control)
*
* Authors: Gregory Stewart, John Estafanos, Andrew Kennah, Patrick Laforest
* Creation Date: Jan 22, 2024
* Last Update: Jan 26, 2024
* Board: Uno R4 Minima
* Version: 3
*
I/O:
- force sensor: digital inputs
- enable: pwm digital output
- voltage sensor: analog input
- current sensor: analog input
- direction: digital outputs
- potentiometer: manual control of PWM or current setpoint

Description:
- This code is written to test the rack and pinion mechanism with a 390 motor driven back and forth. 
- There are force (strain gauge), voltage and current sensors.

Functions & Descriptions:
- GetCurrentValue(): Runs code to read the current sensor and translate its output into amps.
- ReadSensors(): Runs the current value acquisition function and also obtains the strain gauge value and the current.
- CCUpdatePWM(): Uses a controller to output an appropriate correction to the PWM based on current value error.
- CalibrateCurrent(): In the setup, takes a few iterations to record the current input at 0A so that it may be used for correction.

References:
- Link: https://forum.arduino.cc/t/is-there-a-good-replacement-for-timerone/1182956/10
*/



//---------------------------------------------------------------------


// Libraries
#include "DueTimer.h"
#include "HX711.h"
#include <PID_v1.h>

// Pins
#define potPin A0
#define sensV A1
#define sensC A2  //current is analog signal via 100mV/A
#define mdEn 3 //motor driver enable, PWM signal
#define mdIn1 8 //high or low signal for direction, complements mdIn2
#define mdIn2 9 //low or high signal for direction, complements mdIn1
#define led LED_BUILTIN

// Misc.
String runMode = "run"; // can be 'run' or 'stop' or 'poten'
int potVal; // for potentiometer manual PWM control
int sensVVal = 0;
int sensCVal = 0;
volatile int cnt = 0;
unsigned long scaledVolt;
float realVolt = 0;
float realAmp = 0;
double currentSensorSensitivity = 66; // mv/A
int pwmCeiling = 80;

// LED control (for PWM state visualization)
int in1State = LOW;
int in2State = LOW;
int ledState = LOW;

// Strain gauge pins and object
const int LOADCELL_DOUT_PIN = 5;
const int LOADCELL_SCK_PIN = 6;
long reading;
long lastReading;
unsigned long dReading;
float weight;
HX711 scale;  // object for strain gauge

// Current sensing
unsigned int total; // holds <= 64 analogReads
byte numReadings = 64;
float offset = 512.1; // calibrate zero current
float span = 0.066; // calibrate max current | ~0.07315 is for 30A sensor
double current; // holds final current, in A
float rawData;

// Current control
int pwm = 25;
int duty;
int setT = 10; // 23 // in N*m
double k_IT = 0.1282; // in A/(N*m)
double setI = 4.00; // in A
float deltaI;
float torque; // in N*m

// PID
double setIPID, currentPID, outPID; // define PID variable
double Kp=2, Ki=0, Kd=0;  // specify initial tuning parameters
int errPWM;
// PID myPID(&currentPID, &outPID, &setIPID, Kp, Ki, Kd, DIRECT);
PID myPID(&current, &outPID, &setI, Kp, Ki, Kd, DIRECT);
double maxCorrect = pwmCeiling;

// Current sensor offset correction
float currentOffset = 0;  // in A
int offsetWindow = 1000;
float currentSum;

// Set up PWM offset for PID use
int pwmOffset = 0;
float resistance = 0.2234;  // in ohm
float setV = 0;



//---------------------------------------------------------------------


void GetCurrentValue() {
  total = 0; // reset
  for (int i = 0; i < numReadings; i++) total += analogRead(sensC);
  current = (total / numReadings - offset) * span;
}

void ReadSensors() {
  // // Check strain gauge value & calculate weight
  // if (scale.is_ready()) {
  //   lastReading = reading;
  //   reading = scale.read();
  //   reading = -reading - 1500000;
  // } else {
  //   // Serial.println("HX711 not found.");
  // }
  // dReading = reading - lastReading;
  // if (dReading > 0) {
  //   weight = dReading / 45 / 1000;  // 45 units measured are approx 1g, weight is in kg
  // }

  // Check voltage sensor
  sensVVal = analogRead(sensV);
  scaledVolt = map(sensVVal, 0, 1023, 0, 5000);
  realVolt = scaledVolt * 0.005;  // in volts

  // Check current sensor
  GetCurrentValue();
  current -= (currentOffset-7.24);

  CCUpdatePWM();
}

void CCUpdatePWM() {
  // Current control (PID library)
  currentPID = current;
  setIPID = setI;
  myPID.Compute();
  pwmOffset = int(setV / realVolt * 255.0);
  // pwm = int((double(pwm)*100.0 + outPID*100.0)) / 100;
  pwm = pwmOffset + int(outPID);
  if (pwm > pwmCeiling) {pwm = pwmCeiling;} else if (pwm < 0) {pwm = 0;}
  analogWrite(mdEn, pwm);

  // // Current control (not PID)
  // deltaI = setI - current;
  // pwm = pwm + deltaI*6;
  // if (pwm > pwmCeiling) {pwm = pwmCeiling;} else if (pwm < 0) {pwm = 0;}
  // analogWrite(mdEn, pwm);
}

void CalibrateCurrent() {
  for (int i = 0; i < offsetWindow; i++)
  {
    GetCurrentValue();
    currentSum += current;
    if (i == (offsetWindow-1)) {
      currentOffset = currentSum / float(offsetWindow-1);
    }
  }
  
}


//---------------------------------------------------------------------


void setup() {
  Serial.begin(115200);
  pinMode(sensV, INPUT);
  pinMode(sensC, INPUT);
  pinMode(mdEn, OUTPUT);
  pinMode(mdIn1, OUTPUT);
  pinMode(mdIn2, OUTPUT);
  pinMode(led, OUTPUT);
  Timer3.attachInterrupt(ReadSensors).start(1000);
  digitalWrite(led, LOW);
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN, 128);
  myPID.SetOutputLimits(-maxCorrect, maxCorrect);
  myPID.SetMode(AUTOMATIC);
  CalibrateCurrent();
  delay(1500);
  Serial.println("Starting now.");
  digitalWrite(mdIn1, LOW);
  digitalWrite(mdIn2, HIGH);
  setV = resistance * setI;
}

void loop() {
  if(runMode == "run") {
    if (current < 15) {
      // Read sensors and update PWM for current control accordingly
      digitalWrite(led, HIGH);

      // Print values here, then record using Realterm and process using Excel
      deltaI = setI - current;
      Serial.print(millis());
      Serial.print(",");
      Serial.print(deltaI);     // in A
      Serial.print(",");
      Serial.print(setI);       // in A
      Serial.print(",");  
      Serial.print(reading);    // in kg
      Serial.print(",");
      Serial.print(realVolt);   // in V
      Serial.print(",");
      Serial.print(current);    // in A
      Serial.print(",");
      Serial.print(pwm);        // 0-255
      Serial.print(",");
      Serial.print(pwmOffset);
      Serial.print(",");
      Serial.println(outPID);
    } else {
      analogWrite(mdEn, 0);
    }
  } else if (runMode == "stop") {
    analogWrite(mdEn, 0);
  } else if (runMode == "poten") {
    digitalWrite(led, HIGH);
    potVal = map(analogRead(potPin), 0, 1023, 0, pwmCeiling);  // setting PWM ceiling at 200 (max 255)
    analogWrite(mdEn, potVal);
    ReadSensors();
  }
}

