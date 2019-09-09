#include "MotorManager.h"
#include <Arduino.h>

MotorManager::MotorManager(functiontypeint leftForward, functiontypeint leftReverse,
                functiontypeint rightForward, functiontypeint rightReverse, functiontype stop)
{
    _leftForward = leftForward;
    _leftReverse = leftReverse;
    _rightForward = rightForward;
    _rightReverse = rightReverse;
    _stop = stop;
}

void MotorManager::setSpeedLimits(int min, int max)
{
    _min = min;
    _max = max;
}

int MotorManager::getMinMotorSpeed()
{
    return _min;
}

int MotorManager::getMaxMotorSpeed()
{
    return _max;
}

void MotorManager::leftForward(int speed)
{
    // Convert speed in percentag to value
    int speedVal = map(speed, 0, 100, _min, _max);
    // Serial.print("LF:\t"); Serial.print(speedVal); Serial.print("\t");
    _leftForward(speedVal);
}

void MotorManager::leftReverse(int speed)
{
    // Convert speed in percentag to value
    int speedVal = map(speed, 0, 100, _min, _max);
    _leftReverse(speedVal);
}

void MotorManager::rightForward(int speed)
{
    // Convert speed in percentag to value
    int speedVal = map(speed, 0, 100, _min, _max);
    // Serial.print("RF:\t"); Serial.println(speedVal);
    _rightForward(speedVal);
}

void MotorManager::rightReverse(int speed)
{
    // Convert speed in percentag to value
    int speedVal = map(speed, 0, 100, _min, _max);
    _rightReverse(speedVal);
}

void MotorManager::stop()
{
    _stop();
}
