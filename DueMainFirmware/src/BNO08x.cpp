/* 
* Sys-MoDEL Active Suspension BNO08x.cpp file
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

#include "BNO08x.h"

// Forward declaration of Offsets_IMU struct
struct Offsets_IMU;

BNO08xIMU::BNO08xIMU()
{
    _bno08x = new Adafruit_BNO08x();
}

void BNO08xIMU::SetReports(sh2_SensorId_t reportType, long report_interval) 
{
    _bno08x->enableReport(_reportTypePitchRoll, _reportIntervalUs);
    _bno08x->enableReport()
}

void BNO08xIMU::QuaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees)
{
    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    ypr->_yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr->_pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr->_roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    if (degrees) {
      ypr->_yaw *= RAD_TO_DEG;
      ypr->_pitch *= RAD_TO_DEG;
      ypr->_roll *= RAD_TO_DEG;
    }
}

void BNO08xIMU::QuaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees) 
{
    QuaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void BNO08xIMU::BeginBNO08x()
{
    _bno08x->begin_I2C();
    SetReports(_reportType, _reportIntervalUs);
}

double BNO08xIMU::GetPitch()
{
    sh2_SensorValue_t sensorValue;
    if (_bno08x->wasReset())
    {
        SetReports(_reportType, _reportIntervalUs);
    }

    if (_bno08x->getSensorEvent(&sensorValue))
    {
        QuaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &_ypr, true);
    }
    return _ypr._pitch;
}

double BNO08xIMU::GetRoll()
{
    sh2_SensorValue_t sensorValue;
    if (_bno08x->wasReset())
    {
        SetReports(_reportType, _reportIntervalUs);
    }

    if (_bno08x->getSensorEvent(&sensorValue))
    {
        QuaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &_ypr, true);
    }
    return _ypr._roll;
}