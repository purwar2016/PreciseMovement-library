#ifndef _PreciseMover_h
#define _PreciseMover_h

#include <Arduino.h>
#include "DeadReckonerPM.h"
#include "PidPM.h"

typedef void(*functiontype)();

class PreciseMover {

public:
    PreciseMover(
            volatile unsigned long *newPulsesLeft,
            volatile unsigned long *newPulsesRight,
            int newPulsesPerRev,
            int newEnA,
            int newEnB,
            double newLength,
            double newRadius,
            int newMinMotorSpeed,
            int newTargetForwardSpeed,
            double newStopLength,
            int targetTwistOmega,
            int angleErrorThreshold,
            functiontype newMotorSetForward,
            functiontype newMotorSetCCW,
            functiontype newMotorSetCW,
            functiontype motorBrake,
            double newKpFW,
            double newKiFW,
            double newKpTW,
            double newKiTW);

    void forward(double);
    void twist(double);

    void setTuneDelay(int);

    void tuneTwistPID();
    void debug();

private:
    volatile unsigned long *pulsesLeft, *pulsesRight;
    double LENGTH;
    double RADIUS;
    double MIN_MOTOR_SPEED; // in PWM
    int PULSES_PER_REV;

    double ENA;
    double ENB;

    double PID_SAMPLE_TIME = 15;
    double PID_MIN_FW = -100;
    double PID_MAX_FW = 100;
    double kpFW;
    double kiFW;
    double kdFW;
    double inputFW;
    double outputFW;
    double setpointFW;
    double PID_MIN_TW = 100;
    double PID_MAX_TW = 255;
    double kpTW;
    double kiTW;
    double kdTW;
	double inputLeftTW, inputRightTW;
	double outputLeftTW, outputRightTW;
	double setpointLeftTW, setpointRightTW;

    double TARGET_FORWARD_SPEED;
    double TARGET_TWIST_MOTOR_OMEGA;

    double ANGLE_ERROR_THRESHOLD;
    double STOP_LENGTH;
    double LOOK_AHEAD = 100;

    unsigned long POSITION_COMPUTE_INTERVAL = 25; // ms
    unsigned long CORRECTION_TIME_INTERVAL_FW = 50;
    unsigned long CORRECTION_TIME_INTERVAL_TW = 25;

    unsigned long prevPositionComputeTime = 0;
    unsigned long prevCorrectionTime = 0;

    functiontype motorSetForward;
    functiontype motorSetCCW;
    functiontype motorSetCW;
    functiontype motorBrake;

    DeadReckonerPM deadReckoner;
    PidPM pidFW;
	PidPM pidLeftMotorTW;
	PidPM pidRightMotorTW;

    void doDeadReckoning();
    double getGoalPointX(double xPos, double yPos, double xTarget);
    void moveWheels(int motorSpeed, double diff);

    void printPlot(int actual, int target);

    unsigned long tuneDelay = 1000; // ms
};

#endif
