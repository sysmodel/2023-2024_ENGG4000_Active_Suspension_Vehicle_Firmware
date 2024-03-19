#ifndef FFIC_H
#define FFIC_H

//------------------------------------------------------------------

#include "Adafruit_INA260.h"
#include "PID_v1.h"

//------------------------------------------------------------------

void InitFFIC();
void SetDirec(int wheel, bool dir);
float GetCurrent(int wheel);
int CCUpdatePWM(int wheel, float setI, float crnt, float batteryVolt);
void ActuateAction();

//------------------------------------------------------------------

#endif  // FFIC_H