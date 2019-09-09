#include "EncoderManager.h"
#include "FunctionType.h"
#include <Arduino.h>

EncoderManager::EncoderManager(volatile unsigned long* leftEncoderPos, volatile unsigned long* rightEncoderPos, int ticksPerRev)
{
    _leftPos = leftEncoderPos;
    _rightPos = rightEncoderPos;
    _ticksPerRev = ticksPerRev;
}

volatile unsigned long* EncoderManager::getLeftPosPointer()
{
    return _leftPos;
}

volatile unsigned long* EncoderManager::getRightPosPointer()
{
    return _rightPos;
}

int EncoderManager::getTicksPerRev()
{
    return _ticksPerRev;
}
