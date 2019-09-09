#ifndef EncoderManager_h
#define EncoderManager_h

#include "FunctionType.h"

// This class manages the encoder data. 
class EncoderManager
{
public:
    EncoderManager(volatile unsigned long* leftEncoderPos, volatile unsigned long* rightEncoderPos, int ticksPerRev);
    volatile unsigned long* getLeftPosPointer();
    volatile unsigned long* getRightPosPointer();
    int getTicksPerRev();

private:
    volatile unsigned long* _leftPos;
    volatile unsigned long *_rightPos;
    int _ticksPerRev;
};

#endif
