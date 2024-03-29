/* 
* Sys-MoDEL Active Suspension SteeringAngle.h file
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

#ifndef SteeringAngle_H
#define SteeringAngle_H

#include <Arduino.h>

class SteeringPOT
{
    public: 
        // Default constructor 
        SteeringPOT(uint8_t dataPinPOT);

        // Initialize Pins
        void initSteeringPOT();

        // AnalogRead data pin and store raw data
        void getRawDataPOT(uint8_t dataPinPOT);

        // Map the raw data to an actual steering angle
        int checkSteeringAngle();

    private:
        // Variables
        uint8_t _dataPinPOT;
        uint32_t _rawDataPOT;



};

#endif