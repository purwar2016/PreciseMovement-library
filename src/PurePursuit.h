

#ifndef PurePursuit_h
#define PurePursuit_h

#include "CatmullRom.h"
#include <Arduino.h>

class PurePursuit
{
public:
    static constexpr int MIN_PATH_LENGTH = 4;
    static constexpr float DEFAULT_INTERPOLATION_STEP = 20; // mm
    PurePursuit(double* xPos, double* yPos, double *heading, double LOOK_AHEAD, unsigned long INTERVAL=50, int STOP_POINT=3);
    void start();
    bool checkStop();
    void compute();
    void setPath(float* pathX, float* pathY, int pathLength);
    void setPath(CatmullRom* path);
    CatmullRom* getPath();
    float* getArrayX();
    float* getArrayY();
    float getGoalX();
    float getGoalY();
    void setInterpolationStep(float interpolationStep);
    void computeNextGoalPoint();
    double getCurvature();
    double static getDistance(float x0, float y0, float x1, float y1);
    void printPath();

private:
    // The camullrom interpolation is done on the path to make path following smoother with less data points.
    // Target interpolation step for catmull interpolation.
    float _interpolationStep;

    //  The path is encapsulated within the CatmullRom class.
    CatmullRom* _path = nullptr;

    // _stop is set to false after path following is complete.
    // _stop is false by default, and set to true when starting path following.
    bool _stop;

    // Computation interval for the pure pursuit algorithm.
    // See compute() method.
    unsigned long _INTERVAL;

    // Last time computer() method was ran in millisecond.
    unsigned long _prevComputeTime;

    // Look ahead pure pursuit algoirthm parameter.
    double _LOOK_AHEAD;

    // Pointer to the x coordinates of the path.
    double* _xPos;

    // Pointer to the y coordinates of the path.
    double* _yPos;

    // Pointer to the robot heading.
    double* _heading;

    // Curvature computed by the pure pursuit algorithm.
    double _curvature;

    // Current goal point x value.
    double _goalX;

    // Current goal point y value.
    double _goalY;
};

#endif
