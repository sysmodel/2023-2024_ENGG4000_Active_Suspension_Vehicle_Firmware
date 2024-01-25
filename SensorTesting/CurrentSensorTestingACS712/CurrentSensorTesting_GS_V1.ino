/* 
* ACS712 Current Sensor Testing
*
* Authors: Gregory Stewart, John Estafanos, Andrew Kennah, Patrick Laforest
* Creation Date: Jan 6, 2024
* Last Update: Jan 11, 2024
*
* Version 1.0
*
* Description: 
* This code is written to test the current sensing capabilites of a 30 amp ACS712 current sensor. The 
* datasheet for this sensor can be found at: https://www.sparkfun.com/datasheets/BreakoutBoards/0712.pdf
*
* Functions & Descriptions: 
* Name: GetCurrentValue()
* Description: This function takes analog readings from the ACS712 out pin in millivolts and converts the
* reading into milliamps.
*
* References:
* The code below was originally posted at the following link before being modified for this project's 
* perticular needs. Link: https://forum.arduino.cc/t/how-to-calibrate-acs712-properly/540323/2
*/

unsigned int total; // holds <= 64 analogReads
byte numReadings = 64;
float offset = 509; // calibrate zero current
float span = 0.066; // calibrate max current | ~0.07315 is for 30A sensor
float current; // holds final current
float rawData;

void setup() 
{
  Serial.begin(115200);
}

void loop() 
{
  // Call function to calculate current value in milliamps
  GetCurrentValue();
  
  delay(500);
}

void GetCurrentValue()
{
  total = 0; // reset
  for (int i = 0; i < numReadings; i++) total += analogRead(A0);
  current = (total / numReadings - offset) * span;
  rawData = analogRead(A0);
  Serial.print("Raw Data: ");
  Serial.print(rawData);
  Serial.print("Current is  ");
  Serial.print(current);
  Serial.println("  Amp");
}