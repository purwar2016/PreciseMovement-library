#ifndef _DeadReckoner_h
#define _DeadReckoner_h
#define UNSIGNED_LONG_MAX 4294967295
#define WHEEL_FORWARD 1
#define WHEEL_REVERSE -1

#include <Arduino.h>

class DeadReckoner {

public:
	static const float RAD_PER_SEC_TO_RPM = 30.0 / PI;
	DeadReckoner();
	DeadReckoner(volatile unsigned long *left, volatile unsigned long *right,
					int tpr, double r, double l, unsigned long ci=50);
	void Init(volatile unsigned long *left, volatile unsigned long *right,
					int tpr, double r, double l, unsigned long ci=50);
	void computePosition();
	void setX(double);
	void setY(double);
	void setHeading(double);
	void setLeftDirection(int);
	void setRightDirection(int);
	int getLeftOmegaDirection();
	int getRightOmegaDirection();
	double getX();
	double getY();
	double* getXPointer();
	double* getYPointer();
	double getW();
	double getWl();
	double getWr();
	double getHeading();
	double* getHeadingPointer();
	void reset();

private:
	void computeAngularVelocities();
	volatile unsigned long *leftTicks, *rightTicks; // Number of total wheel encoder tick counts for left and right wheels.
	unsigned long leftTicksPrev, rightTicksPrev; // Number of total wheel encoder tick counts at time computeAngularVelocities() is called.
	double xc, yc; // Latest position coordinates in ticks.
	double wl, wr; // Latest left and right angular velocity of the wheels in radians per second.
	int ticksPerRev; // Number of tick registers per second of the encoder.
	float w; // Angular velocity of the robot.
	double length; // Length from left wheel to right wheel.
	double radius; // Radius of the wheel.
	double heading;
	double toRadPerSec; // ticks/microsecond to rad/s conversion factor
	unsigned long prevIntegrationTime;
	unsigned long prevWheelComputeTime;
	unsigned long positionComputeInterval;
	int leftOmegaDirection = 1;
	int rightOmegaDirection = 1;
	unsigned long dt_omega;
	unsigned long dt_integration;
	unsigned long static getChange(unsigned long current, unsigned long previous);
};

#endif
