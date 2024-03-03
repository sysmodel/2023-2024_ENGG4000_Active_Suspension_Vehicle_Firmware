#ifndef TEST_HEADER_H
#define TEST_HEADER_H

//------------------------------------------------------------------

#include <Arduino.h>
#include "DueTimer.h"
#include "HX711.h"
#include "PID_v1.h"
#include "Adafruit_INA260.h"

#define potPin A0
#define sensV A1
#define mdEn 3 //motor driver enable, PWM signal
#define mdIn1 8 //high or low signal for direction, complements mdIn2
#define mdIn2 9 //low or high signal for direction, complements mdIn1
#define led LED_BUILTIN

//------------------------------------------------------------------

// Instantiate objects
extern PID myPID;
extern HX711 scale;  // object for strain gauge
extern Adafruit_INA260 ina260;  // object for current sensor

// Voltage sensor
extern float voltage;

// PID
extern int pwmOffset;
extern int pwm;
extern double setIPID, currentPID, outPID; // define PID variable
extern double Kp, Ki, Kd;  // specify PID tuning parameters
extern double maxCorrect;

// Loadcell
extern long readingSG; // strain gauge reading
extern const int LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN; // digital output & SCK pin for HX711

// Current sensor (INA260)
extern float currentINA;
extern float currentOffsetINA;

//------------------------------------------------------------------

void InitStuff();
void GetVoltage();
void SetDirec(String dir);
void GetCurrentINA(float offset);
void CalibrateCurrentINA();

//------------------------------------------------------------------

#endif // TEST_HEADER_H
