#include "CatmullRom.h"
#include <Arduino.h>

CatmullRom::CatmullRom() { }

// Constructor
CatmullRom::CatmullRom(float* pathX, float* pathY, int numPoints)
{
    setPoints(pathX, pathY, numPoints);
}

// Set the data points.
void CatmullRom::setPoints(float* pathX, float* pathY, int numPoints)
{
    _pathX = pathX;
    _pathY = pathY;
    _numPoints = numPoints;
}

// Print path to serial monitor neatly.
void CatmullRom::printPath()
{
    Serial.println("-------------------------------------------------------------");
    Serial.println("PRINTING PATH");
    Serial.println("x\ty\ti");
    for (int i = 0; i < _numPoints; i++)
    {
        Serial.print(_pathX[i]); Serial.print("\t");
        Serial.print(_pathY[i]); Serial.print("\t");
        Serial.println(i);
    }
    Serial.println("PRINT PATH COMPLETE");
    Serial.println("-------------------------------------------------------------");
}

// Next iteration
bool CatmullRom::next()
{
    if (hasNext())
    {
        _t += _parameterStepSize;
        if (_t > 1)
        {
            // Advance one array index.
            _iterationArrayIndex++;
            int stepSize = _iterationArrayIndex == _numPoints - 3 ? END_STEP_SIZE : _stepSize;
            _parameterStepSize = getParameterStep(_iterationArrayIndex, stepSize);
            _t = _parameterStepSize;
        }
        interpolate(_iterationArrayIndex, _t, &_iterationX, &_iterationY);

        // Serial.print(_iterationX); Serial.print("\t");
        // Serial.print(_iterationY); Serial.print("\t");
        // Serial.print(_parameterStepSize); Serial.print("\t");
        // Serial.print(_t); Serial.print("\t");
        // Serial.println(_iterationArrayIndex);

        return true;
    }
    return false;
}

// Previous iteration
bool CatmullRom::prev()
{
    if (hasPrev())
    {
        _t -= _parameterStepSize;
        if (_t < 0)
        {
            // Back one array index
            _iterationArrayIndex--;
            _parameterStepSize = getParameterStep(_iterationArrayIndex, _stepSize);
            _t = floor(1.0 / _parameterStepSize) * _parameterStepSize;
        }
        else if (_t == 0)
        {
            return prev();
        }
        interpolate(_iterationArrayIndex, _t, &_iterationX, &_iterationY);

        // Serial.print(_iterationX); Serial.print("\t");
        // Serial.print(_iterationY); Serial.print("\t");
        // Serial.print(_parameterStepSize); Serial.print("\t");
        // Serial.print(_t); Serial.print("\t");
        // Serial.println(_iterationArrayIndex);

        return true;
    }
    return false;
}

// Returns true if next iteration is valid.
bool CatmullRom::hasNext()
{
    if (_t + _parameterStepSize > 1)
    {
        return _iterationArrayIndex + 4 < _numPoints;
    }
    return true;
}

// Returns true if previous iteration is valid.
bool CatmullRom::hasPrev()
{
    if (_t - _parameterStepSize < 0)
    {
        return _iterationArrayIndex > 3;
    }
    return true;
}

// Get the interpolated x value at the current interation.
float CatmullRom::getIterationX()
{
    return _iterationX;
}

// Get the interpolated y value at the current interation.
float CatmullRom::getIterationY()
{
    return _iterationY;
}

// Reset the iteration to the beginning with target stepSize.
// The stepSize is the target distance for the iteration. It's not the t parameter which goes from 0 to 1.
void CatmullRom::resetIterator(float stepSize)
{
    _iterationIndex = 0;
    _iterationArrayIndex = 1;
    _t = 0;
    _stepSize = stepSize;
    _parameterStepSize = getParameterStep(_iterationArrayIndex, _stepSize);
    interpolate(_iterationArrayIndex, _t, &_iterationX, &_iterationY);
}

// Get the x coordinate in the x path array.
float CatmullRom::getX(int i)
{
    return _pathX[i];
}

// Get the y coordinate in the y path array.
float CatmullRom::getY(int i)
{
    return _pathY[i];
}

float* CatmullRom::getArrayX()
{
    return _pathX;
}

float* CatmullRom::getArrayY()
{
    return _pathY;
}

int CatmullRom::getLength()
{
    return _numPoints;
}

float CatmullRom::getDistance(float x1, float y1, float x2, float y2)
{
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

// Perform catmullRomInterpolation at the ith point on ary using parameter t.
float CatmullRom::catmullRomInterpolation(float* ary, int i, float t)
{
    return 0.5 * (t * t * t * (ary[i + 2] - 3 * ary[i + 1] + 3 * ary[i] - ary[i - 1]) + t * t *
            (-ary[i + 2] + 4 * ary[i + 1] - 5 * ary[i] + 2 * ary[i - 1]) + t * (ary[i + 1] - ary[i - 1]) + 2 * ary[i]);
}

// Perform catmull interolation on the ith point on the x and y array and store the result.
void CatmullRom::interpolate(int i, float t, float* x, float* y)
{
    if (i + 2 <= _numPoints)
    {
        *x = catmullRomInterpolation(_pathX, i, t);
        *y = catmullRomInterpolation(_pathY, i, t);
    }
    else
    {
        // Invalid input.
        // Serial.println("ERROR: interpolate");
    }
    
}

// Convert the targetStep to the t parameter (0 to 1).
float CatmullRom::getParameterStep(int i, float targetStep)
{
    if (i + 1 <= _numPoints)
    {
        float distance = getDistance(_pathX[i], _pathY[i], _pathX[i + 1], _pathY[i + 1]);
        float step = targetStep / distance;
        return step < 1 ? step : 1;
    }
    else
    {
        // Invalid input
        // Serial.println("ERROR: getParameterStep");
        return -1;
    }
    
}
