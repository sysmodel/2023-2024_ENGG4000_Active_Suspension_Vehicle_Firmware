/* 
* Sys-MoDEL Active Suspension VCNL4040.h file
*
* Authors: Gregory Stewart, John Estafanos, Andrew Kennah, Patrick Laforest
* Creation Date: Jan 6, 2024
* Last Update: March 6, 2024
*
* Version 1.0
*
* Description: 
*
* Functions & Descriptions: 
* Name: 
* Description: 
*
* Name: 
* Description: 
* References:
*
*/

#include "VCNL4040.h"

VCNL4040IR::VCNL4040IR() 
{
    _vcnl4040 = new Adafruit_VCNL4040();
}

bool VCNL4040IR::Begin()
{
    _vcnl4040->begin();
    _vcnl4040->setProximityLEDCurrent(VCNL4040_LED_CURRENT_200MA);
    _vcnl4040->setAmbientIntegrationTime(VCNL4040_AMBIENT_INTEGRATION_TIME_320MS);
    _vcnl4040->setProximityIntegrationTime(VCNL4040_PROXIMITY_INTEGRATION_TIME_8T);
}

double VCNL4040IR::GetDistance()
{
    _rawReading = _vcnl4040->getProximity();
    return _factor * pow(_rawReading, _exponent);
}

VCNL4040IR::~VCNL4040IR() 
{
    delete _vcnl4040;
}