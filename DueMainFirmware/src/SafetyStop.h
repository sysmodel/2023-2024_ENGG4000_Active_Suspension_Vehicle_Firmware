/* 
* Sys-MoDEL Active Suspension QuadEncoder_M.h file
*
* Authors: Gregory Stewart, John Estafanos, Andrew Kennah, Patrick Laforest
* Creation Date: Mar 23, 2024
* Last Update: Mar 23, 2024
*
* Version 1.0
*
* Description: 
* This code is the SafetyStop.cpp file that is used to check some conditions 
* to ensure the safety of the vehicle. The output of this script may stop the
* running of the motors if conditions are not satisfactory.
*
* Functions & Descriptions: 
* Name: CheckStopCondition()
* Description: Checks conditions and outputs check results
*
* References:
*
*/

#ifndef SAFETYSTOP_H
#define SAFETYSTOP_H

#include <Arduino.h>

// Stop condition codes
#define BATT_VOLTAGE_TOO_LOW 1
#define FR_CURRENT_TOO_HIGH 2
#define FL_CURRENT_TOO_HIGH 3
#define BR_CURRENT_TOO_HIGH 4
#define BL_CURRENT_TOO_HIGH 5
#define FR_ABSENC_TOO_FAR 6
#define FL_ABSENC_TOO_FAR 7
#define BR_ABSENC_TOO_FAR 8
#define BL_ABSENC_TOO_FAR 9
#define MANUAL_STOP 10


bool CheckStopCondition(float *cellVolt, float *curr, double *encPos, bool sw);


#endif // SAFETYSTOP_H