/* 
* Active Suspension Project Main Firmware of Arduino Due Board
*
* Authors: Gregory Stewart, John Estafanos, Andrew Kennah, Patrick Laforest
* Creation Date: Jan 16, 2024
* Last Update: Jan 16, 2024
* Board: Arduino Due
* Version: 1
*
* I/O:
* - enable: pwm digital output
* - direction: digital outputs
* - voltage sensor: analog input
* - current sensor: analog input
* - I2C communication: VCNL4040, BNO085, Jetson Nano (?)
*
* Description: 
* This code is written to test the rack and pinion mechanism with a 390 motor driven back and forth. 
* There are force (strain gauge), voltage and current sensors.
*
* Functions & Descriptions: 
* - Name: 
* -- Description: 
*
* References:
* None yet.
*/

//---------------------------------------------------------------------

// Libraries
#include "DueTimer.h"
#include <Wire.h>

// Pins
#define batt1Cell1 A0 // voltage sensors for battery cell monitoring
#define batt1Cell2 A1
#define batt2Cell1 A2
#define batt2Cell2 A3
#define amp_FR A4 // current sensors to find motor applied torque
#define amp_FL A5
#define amp_RL A6
#define amp_RR A7
#define mdEn_FR 2 // motor driver enable, PWM signal
#define mdIn1_FR 3  // high or low signal for direction, complements mdIn2
#define mdIn2_FR 4  // low or high signal for direction, complements mdIn1
#define mdEn_FL 5
#define mdIn1_FL 6
#define mdIn2_FL 7
#define mdEn_RL 8
#define mdIn1_RL 9
#define mdIn2_RL 10
#define mdEn_RR 11
#define mdIn1_RR 12
#define mdIn2_RR 13
#define led LED_BUILTIN // built-in LED can be used to visualize state of operation

// --- Variables ---
// Current sensor
unsigned int total_FR; // holds <= 64 analogReads
unsigned int total_FL;
unsigned int total_RL;
unsigned int total_RR;
byte numReadings = 64;
float span = 0.066; // calibrate max current | ~0.07315 is for 30A sensor
float offset_FR = 512.1; // calibrate for analog signal at zero current
float offset_FL = 512.1;
float offset_RL = 512.1;
float offset_RR = 512.1;
float current_FR; // holds final current
float current_FL;
float current_RL;
float current_RR;
float rawData;
// Voltage sensor
unsigned long scaledVolt_11;
unsigned long scaledVolt_12;
unsigned long scaledVolt_21;
unsigned long scaledVolt_22;
float realVolt_11 = 0;
float realVolt_12 = 0;
float realVolt_21 = 0;
float realVolt_22 = 0;
// LED control (for PWM state visualization)
int in1State = LOW;
int in2State = LOW;
int ledState = LOW;
// Miscellaneous
float gearRatio = 16.4;

//---------------------------------------------------------------------

void InitPins() {
  pinMode(batt1Cell1, INPUT);
  pinMode(batt1Cell2, INPUT);
  pinMode(batt2Cell1, INPUT);
  pinMode(batt2Cell2, INPUT);
  pinMode(mdEn_FR, OUTPUT);
  pinMode(mdIn1_FR, OUTPUT);
  pinMode(mdIn2_FR, OUTPUT);
  pinMode(mdEn_FL, OUTPUT);
  pinMode(mdIn1_FL, OUTPUT);
  pinMode(mdIn2_FL, OUTPUT);
  pinMode(mdEn_RL, OUTPUT);
  pinMode(mdIn1_RL, OUTPUT);
  pinMode(mdIn2_RL, OUTPUT);
  pinMode(mdEn_RR, OUTPUT);
  pinMode(mdIn1_RR, OUTPUT);
  pinMode(mdIn2_RR, OUTPUT);
  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);
}

void GetCurrentValues() {
  total = 0; // reset
  for (int i = 0; i < numReadings; i++) {
    total_FR += analogRead(amp_FR);
    total_FL += analogRead(amp_FL);
    total_RL += analogRead(amp_RL);
    total_RR += analogRead(amp_RR);
  }
  current_FR = (total_FR / numReadings - offset_FR) * span;
  current_FL = (total_FL / numReadings - offset_FL) * span;
  current_RL = (total_RL / numReadings - offset_RL) * span;
  current_RR = (total_RR / numReadings - offset_RR) * span;
  // rawData = analogRead(A0);
  // Serial.println(current);  // in amps
}

void ReadVoltages() {
  scaledVolt_11 = map(analogRead(batt1Cell1), 0, 1023, 0, 5000);
  scaledVolt_12 = map(analogRead(batt1Cell2), 0, 1023, 0, 5000);
  scaledVolt_21 = map(analogRead(batt2Cell1), 0, 1023, 0, 5000);
  scaledVolt_22 = map(analogRead(batt2Cell2), 0, 1023, 0, 5000);
  realVolt_11 = scaledVolt_11 * 0.005;  // in volts
  realVolt_12 = scaledVolt_12 * 0.005;  // in volts
  realVolt_21 = scaledVolt_21 * 0.005;  // in volts
  realVolt_22 = scaledVolt_22 * 0.005;  // in volts
}

//---------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  Wire1.begin();
  InitPins();

}

void loop() {
  ReadVoltages();
  GetCurrentValues();
  // * Note: Next, should add the reading of voltage and current sensors to a timer.
  // * Note: Output of PWM and digital signals should also be in a timer.

}
