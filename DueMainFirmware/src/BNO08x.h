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
        // Constructor 
        BNO08xIMU();

        // Destructor
        // ~BNO08xIMU();

        // Struct for yaw, pitch, and roll
        struct euler_t {
        float _yaw;
        float _pitch;
        float _roll;
        } _ypr;

        // Structure for calibration offsets
        struct Offsets_IMU {
        double _pitchOffset;
        double _rollOffset;
        } _IMUoff;

        // SH2 Report setting protocol function
        void SetReports(sh2_SensorId_t reportType, long report_interval);

        // Begin
        void BeginBNO08x();

        // Math
        void QuaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees);

        // Math
        void QuaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees);

        // Get pitch position function
        double GetPitch();

        // Get roll position function
        double GetRoll();

        // Get pitch and roll rates function
        double GetPitchAndRollRates();
    
    private: 
        // Create object
        Adafruit_BNO08x  *_bno08x;

        // Variables for calibration
        double _sumYaw = 0.0;
        double _sumPitch = 0.0;
        double _sumRoll = 0.0;


        // Variables for report types
        sh2_SensorId_t _reportTypePitchRoll = SH2_ARVR_STABILIZED_RV;
        long _reportIntervalUs = 5000;

};

#endif