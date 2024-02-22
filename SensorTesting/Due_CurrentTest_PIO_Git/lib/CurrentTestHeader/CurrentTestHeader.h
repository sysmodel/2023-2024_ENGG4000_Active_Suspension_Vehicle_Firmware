#ifndef TEST_HEADER_H
#define TEST_HEADER_H

//------------------------------------------------------------------

#include <Arduino.h>
#include "DueTimer.h"
#include "HX711.h"
#include "PID_v1.h"
#include "ADS1X15.h"
#include "Adafruit_INA260.h"

#define potPin A0
#define sensV A1
#define mdEn 3 //motor driver enable, PWM signal
#define mdIn1 8 //high or low signal for direction, complements mdIn2
#define mdIn2 9 //low or high signal for direction, complements mdIn1
#define led LED_BUILTIN
#define ADCCurrent 3 // for pin number on ADC device

//------------------------------------------------------------------

extern PID myPID;
extern ADS1115 ADS;
extern HX711 scale;  // object for strain gauge
extern Adafruit_INA260 ina260;  // object for current sensor

extern double currentOffset;
extern double current; // non-filtered current
extern int csAnalog; // current sensor analog
extern float voltage;
extern double fcurrent; // filtered current
extern int pwmOffset;
extern int pwm;
extern double setIPID, currentPID, outPID; // define PID variable
extern float timeConstant; // time constant in s
extern double Kp, Ki, Kd;  // specify PID tuning parameters
extern long readingSG; // strain gauge reading
extern double maxCorrect;
extern const int LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN; // digital output & SCK pin for HX711

extern String direction;

extern float csVolt;

extern float currentINA;
extern float currentOffsetINA;

//------------------------------------------------------------------

void InitStuff();
void GetCurrent(double currOff);
void CalibrateCurrent();
void GetVoltage();
void GetFilteredCurrent();
void SetDirec(String dir);
void GetCurrentINA(float offset);
void CalibrateCurrentINA();

//------------------------------------------------------------------

#endif // TEST_HEADER_H
