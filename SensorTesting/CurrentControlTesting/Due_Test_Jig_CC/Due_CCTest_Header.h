#ifndef Due_CCTest_H
#define Due_CCTest_H


//---------------------------------------------------------------------

// *** Libraries ***

#include <Arduino.h>
#include "DueTimer.h"
#include "HX711.h"
#include <PID_v1.h>
#include "ADS1X15.h"
// #include "TestVars.h"


//---------------------------------------------------------------------

// *** Definitions ***

#define potPin A0
#define sensV A1
#define sensC A2  //current is analog signal via 100mV/A
#define mdEn 3 //motor driver enable, PWM signal
#define mdIn1 8 //high or low signal for direction, complements mdIn2
#define mdIn2 9 //low or high signal for direction, complements mdIn1
#define led LED_BUILTIN
// #define pwmCeiling 80
// #define offsetWindow 1000


//---------------------------------------------------------------------

// *** Global Variables ***

// Misc.
extern int sensVVal = 0;
extern int sensCVal = 0;
extern unsigned long scaledVolt;
extern float realVolt = 0;
extern float realAmp = 0;
extern double currentSensorSensitivity = 66; // mv/A
extern const int pwmCeiling = 80;

// Strain gauge pins
extern const int LOADCELL_DOUT_PIN = 5;
extern const int LOADCELL_SCK_PIN = 6;
extern long reading;
extern long lastReading;
extern unsigned long dReading;
extern float weight;
// extern HX711 scale;  // object for strain gauge

// Current sensing
extern unsigned int total; // holds <= 64 analogReads
extern byte numReadings = 64;
extern float offset = 512.1; // calibrate zero current
extern float span = 0.066; // calibrate max current | ~0.07315 is for 30A sensor
extern double current; // holds final current, in A

// Current control
extern int pwm = 25;
extern double k_IT = 0.1282; // in A/(N*m)
extern double setI = 4.00; // in A
extern float deltaI;
extern float torque; // in N*m

// PID
extern double setIPID, currentPID, outPID; // define PID variable
extern double Kp=2, Ki=0, Kd=0;  // specify initial tuning parameters
extern int errPWM;
// PID myPID(&currentPID, &outPID, &setIPID, Kp, Ki, Kd, DIRECT);
// PID myPID(&current, &outPID, &setI, Kp, Ki, Kd, DIRECT);
extern double maxCorrect = pwmCeiling;

// Current sensor offset correction
extern float currentOffset = 0;  // in A
extern int offsetWindow = 1000;
extern float currentSum;

// Set up PWM offset for PID use
extern int pwmOffset = 0;
extern float setV = 0;


//---------------------------------------------------------------------

// *** Function Declarations ***

void GetCurrentValue();
void ReadSensors();
void CCUpdatePWM();
void CalibrateCurrent();


//---------------------------------------------------------------------


#endif // Due_CCTest_H