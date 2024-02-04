#include "ADS1X15.h"
#include "DueTimer.h"


ADS1115 ADS(0x48);


unsigned int total; // holds <= 64 analogReads
byte numReadings = 64;
float offset = 495; // calibrate zero current
float span = 0.066; // calibrate max current | ~0.07315 is for 30A sensor
double current; // holds final current, in A
double fcurrent; // filtered current
double gcurrent;
float timeConstant = 0.04; // time constant in s
long int nowtime = 0;
long int lasttime = 0;
int csVoltage;
double dt;
double test;
int16_t val_3;
float f;

void FilterOutput() {
  // int16_t val_3 = ADS.readADC(3);  
  // float f = ADS.toVoltage(1);  // voltage factor
  // csVoltage = int(val_3 * f * 1000.0);
  current = (csVoltage - 2460)/(0.066);
  lasttime = nowtime;
  nowtime = millis();
  fcurrent = fcurrent + (current - fcurrent + 0.0) * double(nowtime - lasttime) / double(1000 * timeConstant);
  dt = nowtime -lasttime;
  test = double(nowtime - lasttime) / double(1000 * timeConstant);
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  // Serial.println(__FILE__);
  // Serial.print("ADS1X15_LIB_VERSION: ");
  // Serial.println(ADS1X15_LIB_VERSION);
  Wire.begin();
  ADS.begin();
  Timer1.attachInterrupt(FilterOutput).start(1000);
  Serial.println("Starting in 1s.");
  delay(1000);


}
 
void loop() {
  // put your main code here, to run repeatedly:
  ADS.setGain(0);
 
  // int16_t val_0 = ADS.readADC(0);  
  // int16_t val_1 = ADS.readADC(1);  
  // int16_t val_2 = ADS.readADC(2);  

  int16_t val_3 = ADS.readADC(3);  
  float f = ADS.toVoltage(1);  // voltage factor
  csVoltage = int(val_3 * f * 1000.0);
  int analog = map(csVoltage, 0, 5000, 0, 1023);
  
 
  // Serial.print("\tAnalog0: "); Serial.print(val_0); Serial.print('\t'); Serial.println(val_0 * f, 3);
  // Serial.print("\tAnalog1: "); Serial.print(val_1); Serial.print('\t'); Serial.println(val_1 * f, 3);
  // Serial.print("\tAnalog2: "); Serial.print(val_2); Serial.print('\t'); Serial.print(val_2 * f, 3);
  // Serial.print("\tAnalog3: "); Serial.print(val_3); Serial.print('\t'); Serial.println(val_3 * f, 3);
  // Serial.println();


  total = 0; // reset
  for (int i = 0; i < numReadings; i++) total += analog;
  double gcurrent = (total / numReadings - offset) * span;

  


  // Serial.print(analog);
  // Serial.print(" , ");
  // Serial.print(csVoltage);
  // Serial.print(" , ");
  // Serial.print(dt);
  // Serial.print(" , ");
  // Serial.print(test);
  // Serial.print(" , ");
  Serial.print(csVoltage);
  Serial.print(" , ");
  Serial.print(analog);
  Serial.print(" , ");
  Serial.print(current);
  Serial.print(" , ");
  Serial.print(fcurrent);
  Serial.print(" , ");
  Serial.println(gcurrent);
 
  // delay(100);
}
 