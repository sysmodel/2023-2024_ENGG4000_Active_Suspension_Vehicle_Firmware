/* 
* Sys-MoDEL Active Suspension SteeringAngle.cpp file
*
* Authors: Gregory Stewart, John Estafanos, Andrew Kennah, Patrick Laforest
* Creation Date: Mar 23, 2024
* Last Update: Mar 23, 2024
*
* Version 1.0
*
* Description: 
* 
*
* Functions & Descriptions: 
* Name: CheckSteeringAngle()
* Description: Checks potentiometer value and calculates the steering angle
*
* References:
*
*/

#include "SteeringAngle.h"

// Actual constructor
SteeringPOT::SteeringPOT(uint8_t dataPinPOT)
{
    _dataPinPOT = dataPinPOT;

    initSteeringPOT();
};

void SteeringPOT::initSteeringPOT()
{
    pinMode(_dataPinPOT, INPUT);
}

void SteeringPOT::getRawDataPOT(uint8_t dataPinPOT)
{
    _dataPinPOT = dataPinPOT;
    _rawDataPOT = analogRead(dataPinPOT);
}

int SteeringPOT::checkSteeringAngle()
{
    getRawDataPOT(_dataPinPOT);

    return map(_dataPinPOT, 0, 1023, -30, 30);
}