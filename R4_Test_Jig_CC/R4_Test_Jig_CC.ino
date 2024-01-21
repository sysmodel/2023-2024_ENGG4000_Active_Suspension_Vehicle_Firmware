/* 
* Rack and Pinion Test Jig Firmware (w/ current control)
*
* Authors: Gregory Stewart, John Estafanos, Andrew Kennah, Patrick Laforest
* Creation Date: Jan 6, 2024
* Last Update: Jan 11, 2024
* Board: Uno R4 Minima
* Version: 1
*
I/O:
- force sensor: digital inputs
- enable: pwm digital output
- voltage sensor: analog input
- current sensor: analog input
- direction: digital outputs

Description: 
- This code is written to test the rack and pinion mechanism with a 390 motor driven back and forth. 
- There are force (strain gauge), voltage and current sensors.

Functions & Descriptions: 
- switchDirecTest(): This function switches the rotational direction of the motor and the PWM (effective voltage) to the motor.
- GetCurrentValue(): Runs code to read the current sensor and translate its output into amps.

References:
- Link: https://forum.arduino.cc/t/is-there-a-good-replacement-for-timerone/1182956/10
*/



//---------------------------------------------------------------------

// Libraries
#include "Timer_AGT_One.h"
#include "HX711.h"

// Pins
#define sensV A1
#define sensC A2  //current is analog signal via 100mV/A
#define mdEn 3 //motor driver enable, PWM signal
#define mdIn1 8 //high or low signal for direction, complements mdIn2
#define mdIn2 9 //low or high signal for direction, complements mdIn1
#define led LED_BUILTIN

int sensVVal = 0;
int sensCVal = 0;
volatile int cnt = 0;
unsigned long scaledVolt;
float realVolt = 0;
float realAmp = 0;
double currentSensorSensitivity = 66; // mv/A
// LED control (for PWM state visualization)
int in1State = LOW;
int in2State = LOW;
int ledState = LOW;
// Strain gauge digital pins
const int LOADCELL_DOUT_PIN = 5;
const int LOADCELL_SCK_PIN = 6;
long reading;
long lastReading;
unsigned long dReading;
float weight;
// Current sensing
unsigned int total; // holds <= 64 analogReads
byte numReadings = 64;
float offset = 512.1; // calibrate zero current
float span = 0.066; // calibrate max current | ~0.07315 is for 30A sensor
float current; // holds final current, in A
float rawData;
// Object for strain gauge
HX711 scale;
// Current control
int pwm = 25;
int duty;
int setT = 20; // in N*m
double k_IT = 0.1282; // in A/(N*m)
float setI;
float deltaI;
float torque; // in N*m

//---------------------------------------------------------------------

void switchDirecTest() {
  if (cnt == 1){
    digitalWrite(mdIn1, HIGH);
    digitalWrite(mdIn2, LOW);
    analogWrite(mdEn, 25);
    ledState = HIGH;
  } else if (cnt == 2) {
    digitalWrite(mdIn1, LOW);
    digitalWrite(mdIn2, HIGH);
    analogWrite(mdEn, 100);
    cnt = 0;
    ledState = LOW;
  }
  cnt++;
  digitalWrite(led, ledState);
}

void GetCurrentValue() {
  total = 0; // reset
  for (int i = 0; i < numReadings; i++) total += analogRead(sensC);
  current = (total / numReadings - offset) * span;
}

void ReadSensors() {

  // Check strain gauge value & calculate weight
  if (scale.is_ready()) {
    lastReading = reading;
    reading = scale.read();
    reading = -reading - 1500000;
  } else {
    Serial.println("HX711 not found.");
  }
  dReading = reading - lastReading;
  if (dReading > 0) {
    weight = dReading / 45 / 1000;  // 45 units measured are approx 1g, weight is in kg
  }

  // Check voltage sensor
  sensVVal = analogRead(sensV);
  scaledVolt = map(sensVVal, 0, 1023, 0, 5000);
  realVolt = scaledVolt * 0.005;  // in volts

  // Check current sensor
  GetCurrentValue();
}

void CCUpdatePWM() {
  digitalWrite(mdIn1, LOW);
  digitalWrite(mdIn2, HIGH);
  if (setI > current) {
    deltaI = setI - current;
    if (delta > 0.5) {
      pwm = pwm + 10;
    } else if (delta > 0.1) {
      pwm = pwm + 5;
    }
  } else if (setI < current) {
    deltaI = current - setI;
    if (delta > 0.5) {
      pwm = pwm - 10;
    } else if (delta > 0.1) {
      pwm = pwm - 5;
    }
  }
  analogWrite(mdEn, pwm);
}


//---------------------------------------------------------------------

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(sensV, INPUT);
  pinMode(sensC, INPUT);
  pinMode(mdEn, OUTPUT);
  pinMode(mdIn1, OUTPUT);
  pinMode(mdIn2, OUTPUT);
  pinMode(led, OUTPUT);
  // Timer1.initialize(1500000);
  // Timer1.attachInterrupt(switchDirecTest);
  digitalWrite(led, LOW);
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN, 128);
  setI = setT * k_IT; // in A
  delay(1500);
}

void loop() {

  // Read sensors and update PWM for current control accordingly
  digitalWrite(led, HIGH);
  ReadSensors();
  CCUpdatePWM();

  // PWM duty cycle (0 to 100)
  duty = pwm / 255 * 100;

  // Torque, based on measured current
  torque = current / k_IT;
  
  // Print values here, then record using Realterm and process using Excel
  Serial.print(weight);     // in kg
  Serial.print(",");
  Serial.print(realVolt);   // in V
  Serial.print(",");
  Serial.print(current);    // in A
  Serial.print(",");
  Serial.print(torque);     // in N*m
  Serial.print(",");
  Serial.println(duty);     // in %
  
  // Artificial delay so there isn't too much data
  delay(100);
}
