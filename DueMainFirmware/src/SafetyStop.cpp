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
float cellVoltLowLimit = 3.4; // in V, true low limit of battery is 3.2V/cell (6.4V)
float currentHighLimit = 10; // administrative limit for high current
int absEncPosLimits[4] = {0,0,0,0}; // absolute encoder limits
byte j = 0; // variable for indexing


int CheckStopCondition(float *cellVolt, float *curr, uint16_t *encPos, bool sw) {

    // Note: If one of the wheels at FL/BR/BR triggers the stop condition, it is ...
    //  ... possible that a preceding location in the loop also crossed the limit. ...
    //  ... However, this function still does its job stopping when a limit is ...
    //  ... crossed at any point. The same is true for the battery cells.

    stopCode = 0;

    // Check battery cell limits
    for(j=0;j<2;j++) {
        if (*(cellVolt+j) < cellVoltLowLimit) {stopCode = BATT_VOLTAGE_TOO_LOW;}
    }

    // Check current limits
    for(j=0;j<4;j++) {
        switch (j) {
            case 0:
                if (*(curr+j) > currentHighLimit) {stopCode = FR_CURRENT_TOO_HIGH;}
                break;
            case 1:
                if (*(curr+j) > currentHighLimit) {stopCode = FL_CURRENT_TOO_HIGH;}
                break;
            case 2:
                if (*(curr+j) > currentHighLimit) {stopCode = BR_CURRENT_TOO_HIGH;}
                break;
            case 3:
                if (*(curr+j) > currentHighLimit) {stopCode = BL_CURRENT_TOO_HIGH;}
                break;
        }
    }

    // Check absolute encoder position limits
    // Note: Configuration and symmetry cause some values to be upper limits ...
    //  ... while others are lower limits.
    for(j=0;j<4;j++) {
        switch (j) {
            case 0:
                if (*(encPos+j) > absEncPosLimits[j]) {stopCode = FR_ABSENC_TOO_FAR;}
                break;
            case 1:
                if (*(encPos+j) < absEncPosLimits[j]) {stopCode = FL_ABSENC_TOO_FAR;}
                break;
            case 2:
                if (*(encPos+j) < absEncPosLimits[j]) {stopCode = BR_ABSENC_TOO_FAR;}
                break;
            case 3:
                if (*(encPos+j) > absEncPosLimits[j]) {stopCode = BL_ABSENC_TOO_FAR;}
                break;
        }
    }

    // Check condition of manual stop switch
    if (sw) {stopCode = MANUAL_STOP;}

    return stopCode;
}