#ifndef TEST_HEADER_H
#define TEST_HEADER_H


#include "DueTimer.h"
#include "HX711.h"
#include <PID_v1.h>
#include "ADS1X15.h"


#define potPin A0
#define sensV A1
#define mdEn 3 //motor driver enable, PWM signal
#define mdIn1 8 //high or low signal for direction, complements mdIn2
#define mdIn2 9 //low or high signal for direction, complements mdIn1
#define led LED_BUILTIN
#define ADCCurrent 3
#define pwmCeiling 50


static int csAnalog; // current sensor analog
static double current; // non-filtered current
static double fcurrent; // filtered current

// static double fcurrent = 5; // filtered current

// struct sensVals {

// }

// class DueTest {
// public:
  
// }

void InitStuff();
double GetCurrent();
double CalibrateCurrent();
float GetVoltage();
int CCUpdatePWM(double setCurr, double curr, float setVolt, float volt);
double GetFilteredCurrent(double fcurrentOffset);


#endif // TEST_HEADER_H