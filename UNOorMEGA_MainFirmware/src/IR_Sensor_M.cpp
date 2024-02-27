/* 
* Sys-MoDEL Active Suspension IR_Sensor.cpp file
*
* Authors: Gregory Stewart, John Estafanos, Andrew Kennah, Patrick Laforest
* Creation Date: Jan 6, 2024
* Last Update: Jan 11, 2024
*
* Version 1.0
*
* Description: 
* This code is the IR_Sensor.cpp file that is used to calculate the vertical distance of the vehicle.  
*
* Functions & Descriptions: 
* Name: 
* Description: 
*
* References:
*
*/

#include "IR_Sensor_M.h"
#include <math.h>

Adafruit_VCNL4040 VCNL4040 = Adafruit_VCNL4040();

void InitializeVCNL4040() 
{
  // Initialize IR sensor for maximum sensitivity at greater ranges
  VCNL4040.setProximityLEDCurrent(VCNL4040_LED_CURRENT_200MA); // Double-check the constant value
  VCNL4040.setProximityLEDDutyCycle(VCNL4040_LED_DUTY_1_40); // Double-check the constant value
  VCNL4040.setAmbientIntegrationTime(VCNL4040_AMBIENT_INTEGRATION_TIME_640MS); // Double-check the constant value
  VCNL4040.setProximityIntegrationTime(VCNL4040_PROXIMITY_INTEGRATION_TIME_8T); // Double-check the constant value
  VCNL4040.setProximityHighResolution(true);
  // Add optional delays between settings if needed
  // delay(50);
}

uint16_t GetProximity() 
{
   uint16_t rawProximity = VCNL4040.getProximity();

  // Convert raw value to double for accurate calculations
  double rawProximityF = (double) rawProximity;

  // Use pow() function for correct exponentiation
  double proximityCM = - 0.2829 * pow(rawProximityF, 3)
                      + 12.481 * pow(rawProximityF, 2)
                      - 181.45 * pow(rawProximityF, 1)
                      + 905.74;

  // Round the calculated centimeters to the nearest integer
  uint16_t proximityInCM = round(proximityCM);

  // Return the rounded proximity value in centimeters
  return proximityInCM;
}
