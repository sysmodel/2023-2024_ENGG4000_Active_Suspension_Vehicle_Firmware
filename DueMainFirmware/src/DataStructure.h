#ifndef DATASTRUCT_H
#define DATASTRUCT_H

#include <Arduino.h>

struct QuarterCar {

    int idx; // will be used to index for the correct pin values
    
    struct AbsoluteEncoder {
        uint16_t Pos;
        double Vel;
        uint8_t sdoPin;
        uint8_t sckPin;
        uint8_t csPin;
    };
    AbsoluteEncoder AbsEnc;

    struct MotorDriver {
        int Pwm;
        int DesDirec;
        uint32_t mdEnPin;
        uint32_t mdIn1Pin;
        uint32_t mdIn2Pin;
    };
    MotorDriver Motor;
    
    struct CurrentSensor {
        float Current;
        int Direc;
    };
    CurrentSensor CurrSens;

    struct Controller {
        float SetI;
        int PwmOffset;
        float resistance;
        float setV;
        double currentPI;
        double outPI;
        double setIPI;
    };
    Controller Control;

};

extern QuarterCar FRqc, FLqc, BRqc, BLqc;

void InitStruct();

#endif // DATASTRUCT_H