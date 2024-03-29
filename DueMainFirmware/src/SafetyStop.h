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


int CheckStopCondition(float cellVolt[2], float curr[4]);


#endif // SAFETYSTOP_H