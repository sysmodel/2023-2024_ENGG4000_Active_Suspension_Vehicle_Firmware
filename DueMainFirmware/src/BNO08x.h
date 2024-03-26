/* 
* Sys-MoDEL Active Suspension BNO08x.h file
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

#ifndef BNO08x_H
#define BNO08x_H

#include <Arduino.h>
#include <Adafruit_BNO08x.h>

class BNO08xIMU
{
    public:
        // Default constructor 
        BNO08xIMU() {}

        // Struct for euler angles
        struct euler_t {
            float yaw;
            float pitch;
            float roll;
        } ypr;

        // Quaternion to Euler Angles
        void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false);

    
    private: 
        // Variables for calibration
        float calibratedYaw = 0.0;
        float calibratedPitch = 0.0;
        float calibratedRoll = 0.0;
        float updatedYaw = 0.0;
        float updatedPitch = 0.0;
        float updatedRoll = 0.0;
        uint16_t calibrationCount = 250;
        uint16_t calibrationFlag = 1;
}

#endif