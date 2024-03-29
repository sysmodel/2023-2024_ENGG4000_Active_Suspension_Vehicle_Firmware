/* 
* Sys-MoDEL Active Suspension QuadEncoder_M.cpp file
*
* Authors: Gregory Stewart, John Estafanos, Andrew Kennah, Patrick Laforest
* Creation Date: Jan 6, 2024
* Last Update: Mar 6, 2024
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


#include "SafetyStop.h"


// Safety stop variables
int stopCode = 0;
float cellVoltLowLimit = 3.1; // in V, true low limit of battery is 3.2V/cell (6.4V)
float currentHighLimit = 10; // administrative limit for high current
byte j = 0; // variable for indexing

// Stop condition codes
int battVoltTooLow = 20;
int currentTooHigh = 30;

int CheckStopCondition(float cellVolt[2], float curr[4]) {

    // Note: If one of the wheels at FL/BR/BR triggers the stop condition, it is ...
    //  ... possible that a preceding location in the loop also crossed the limit. ...
    //  ... However, this function still does its job stopping when a limit is ...
    //  ... crossed at any point. The same is true for the battery cells.

    stopCode = 0;

    // Check battery cell limits
    for(j=0;j<2;j++) {
        if (cellVolt[j] < cellVoltLowLimit) {stopCode = battVoltTooLow + j;}
    }

    // Check current limits
    for(j=0;j<4;j++) {
        if (abs(curr[j]) > currentHighLimit) {stopCode = currentTooHigh + j;}
    }

    return stopCode;
}