#ifndef MotorManager_h
#define MotorManager_h

#include "FunctionType.h"

// This class manages the motors. 
class MotorManager
{
public:
    MotorManager(functiontypeint leftForward, functiontypeint leftReverse,
                functiontypeint rightForward, functiontypeint rightReverse, functiontype stop);
    void setSpeedLimits(int min, int max);
    int getMinMotorSpeed();
    int getMaxMotorSpeed();
    void leftForward(int speed);
    void leftReverse(int speed);
    void rightForward(int speed);
    void rightReverse(int speed);
    void stop();
private:
    int _min = 0;
    int _max = 255;
    functiontypeint _leftForward;
    functiontypeint _leftReverse;
    functiontypeint _rightForward;
    functiontypeint _rightReverse;
    functiontype _stop;
};
#endif
