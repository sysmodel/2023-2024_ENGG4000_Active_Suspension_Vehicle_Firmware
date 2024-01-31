#include "ADS1X15.h"
 
ADS1115 ADS(0x48);


unsigned int total; // holds <= 64 analogReads
byte numReadings = 20;
float offset = 511; // calibrate zero current
float span = 0.066; // calibrate max current | ~0.07315 is for 30A sensor
double current; // holds final current, in A
double fcurrent; // filtered current
float timeConstant = 0.5; // time constant in s
long int time = 0;
long int lasttime = 0;




void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println(__FILE__);
  Serial.print("ADS1X15_LIB_VERSION: ");
  Serial.println(ADS1X15_LIB_VERSION);
  Wire.begin();
  ADS.begin();
  Serial.println("Starting in 1s.");
  delay(1000);


}
 
void loop() {
  // put your main code here, to run repeatedly:
  ADS.setGain(0);
 
  // int16_t val_0 = ADS.readADC(0);  
  // int16_t val_1 = ADS.readADC(1);  
  int16_t val_2 = ADS.readADC(2);  
  int16_t val_3 = ADS.readADC(3);  
 
  float f = ADS.toVoltage(1);  // voltage factor
  // Serial.print(f,3);
  int csVoltage = int(val_3 * f * 1000.0);
  int analog = map(csVoltage, 0, 5000, 0, 1023);
  
 
  // Serial.print("\tAnalog0: "); Serial.print(val_0); Serial.print('\t'); Serial.println(val_0 * f, 3);
  // Serial.print("\tAnalog1: "); Serial.print(val_1); Serial.print('\t'); Serial.println(val_1 * f, 3);
  // Serial.print("\tAnalog2: "); Serial.print(val_2); Serial.print('\t'); Serial.print(val_2 * f, 3);
  // Serial.print("\tAnalog3: "); Serial.print(val_3); Serial.print('\t'); Serial.println(val_3 * f, 3);
  // Serial.println();
  total = 0; // reset
  for (int i = 0; i < numReadings; i++) total += analog;
  current = (total / numReadings - offset) * span;
  current = (csVoltage - 2506)/(0.066);
  lasttime = time;
  time = millis();
  fcurrent = fcurrent + (current - fcurrent + 60.0) * double(time - lasttime) / double(1000 * timeConstant);
  double dt = time -lasttime;
  double test = double(time - lasttime) / double(1000 * timeConstant);
  Serial.print(analog);
  Serial.print(" , ");
  Serial.print(csVoltage);
  Serial.print(" , ");
  Serial.print(dt);
  Serial.print(" , ");
  Serial.print(test);
  Serial.print(" , ");
  Serial.println(fcurrent);
 
  // delay(100);
}
 