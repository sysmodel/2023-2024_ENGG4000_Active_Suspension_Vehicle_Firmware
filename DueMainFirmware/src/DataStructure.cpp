#include "DataStructure.h"


// Motor driver
uint32_t mdEnPins[4] = {4,5,2,3};
uint32_t mdIn1Pins[4] = {51,53,47,49};
uint32_t mdIn2Pins[4] = {50,52,46,48};

// Controller
float resistance[4] = {0.663*(3/6.2)*(3/2.35)*(3/3.3),0.663,0.663,0.663}; // in ohm

// Absolute encoders
uint8_t sdoPin[4] = {11, 13, 7, 9};
uint8_t sckPin[4] = {10, 12, 6, 8};
uint8_t csPin[4] = {25, 24, 27, 26};

QuarterCar FRqc, FLqc, BRqc, BLqc;

void InitStruct() {
    FRqc.idx = 0;
    FLqc.idx = 1;
    BRqc.idx = 2;
    BLqc.idx = 3;
    FRqc.AbsEnc.sdoPin = sdoPin[FRqc.idx];
    FLqc.AbsEnc.sdoPin = sdoPin[FLqc.idx];
    BRqc.AbsEnc.sdoPin = sdoPin[BRqc.idx];
    BLqc.AbsEnc.sdoPin = sdoPin[BLqc.idx];
    FRqc.AbsEnc.sckPin = sckPin[FRqc.idx];
    FLqc.AbsEnc.sckPin = sckPin[FLqc.idx];
    BRqc.AbsEnc.sckPin = sckPin[BRqc.idx];
    BLqc.AbsEnc.sckPin = sckPin[BLqc.idx];
    FRqc.AbsEnc.csPin = csPin[FRqc.idx];
    FLqc.AbsEnc.csPin = csPin[FLqc.idx];
    BRqc.AbsEnc.csPin = csPin[BRqc.idx];
    BLqc.AbsEnc.csPin = csPin[BLqc.idx];
    FRqc.Motor.mdEnPin = mdEnPins[FRqc.idx];
    FLqc.Motor.mdEnPin = mdEnPins[FLqc.idx];
    BRqc.Motor.mdEnPin = mdEnPins[BRqc.idx];
    BLqc.Motor.mdEnPin = mdEnPins[BLqc.idx];
    FRqc.Motor.mdIn1Pin = mdIn1Pins[FRqc.idx];
    FLqc.Motor.mdIn1Pin = mdIn1Pins[FLqc.idx];
    BRqc.Motor.mdIn1Pin = mdIn1Pins[BRqc.idx];
    BLqc.Motor.mdIn1Pin = mdIn1Pins[BLqc.idx];
    FRqc.Motor.mdIn2Pin = mdIn2Pins[FRqc.idx];
    FLqc.Motor.mdIn2Pin = mdIn2Pins[FLqc.idx];
    BRqc.Motor.mdIn2Pin = mdIn2Pins[BRqc.idx];
    BLqc.Motor.mdIn2Pin = mdIn2Pins[BLqc.idx];
    FRqc.Control.resistance = resistance[FRqc.idx];
    FLqc.Control.resistance = resistance[FLqc.idx];
    BRqc.Control.resistance = resistance[BRqc.idx];
    BLqc.Control.resistance = resistance[BLqc.idx];
}