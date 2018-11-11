
#include "DeadReckonerPM.h"
#include <Arduino.h>


DeadReckonerPM::DeadReckonerPM(volatile unsigned long *left, volatile unsigned long *right, int tpr, double r, double l) {
	construct(left, right, tpr, r, l);
}

DeadReckonerPM::DeadReckonerPM() {};

void DeadReckonerPM::construct(volatile unsigned long *left, volatile unsigned long *right, int tpr, double r, double l) {
	leftTicks = left;
	rightTicks = right;
	ticksPerRev = tpr;
	radius = r;
	length = l;
	toRadPerSec = 1000000 * TWO_PI / ticksPerRev;
}

void DeadReckonerPM::setX(double x) {
	xc = x;
}

void DeadReckonerPM::setY(double y) {
	yc = y;
}

void DeadReckonerPM::setTheta(double t) {
	theta = t;
}

double DeadReckonerPM::getX() {
	return xc;
}

double DeadReckonerPM::getY() {
	return yc;
}

double DeadReckonerPM::getWl() {
	// TODO: Check this
	return wl * dt_omega / (UNSIGNED_LONG_MAX - dt_omega);
}

double DeadReckonerPM::getWr() {
	// TODO: Check this
	return wr * dt_omega / (UNSIGNED_LONG_MAX - dt_omega);
}

double DeadReckonerPM::getW() {
	return w * dt_integration / (UNSIGNED_LONG_MAX - dt_integration);
}

double DeadReckonerPM::getTheta() {
	return theta;
}

void DeadReckonerPM::setLeftOmegaDirection(int direction) {
	leftOmegaDirection = direction;
}

void DeadReckonerPM::setRightOmegaDirection(int direction) {
	rightOmegaDirection = direction;
}

int DeadReckonerPM::getLeftOmegaDirection() {
	return leftOmegaDirection;
}

int DeadReckonerPM::getRightOmegaDirection() {
	return rightOmegaDirection;
}

void DeadReckonerPM::computeAngularVelocities() {
	// Time elapsed after computing the angular velocity previously.
	// change in time is defined as previous - current to prevent round off error.
	dt_omega = getChange(prevWheelComputeTime, micros()); // in milliseconds

	double changeLeftTicks = getChange(*leftTicks, leftTicksPrev);
	double changeRightTicks = getChange(*rightTicks, rightTicksPrev);

	wl = leftOmegaDirection * changeLeftTicks / dt_omega * toRadPerSec;
	wr = rightOmegaDirection * changeRightTicks / dt_omega * toRadPerSec;

	//double toRPM = 30.0 / PI;
	//Serial.print("\twl: "); Serial.print(wl*toRPM, 5);
	//Serial.print("\twr: "); Serial.print(wr*toRPM, 5);
	//Serial.print("\tdt: "); Serial.print(dt_omega);
	//Serial.print("\tlt: "); Serial.print(changeLeftTicks);
	//Serial.print("\trt: "); Serial.println(changeRightTicks);

	leftTicksPrev = *leftTicks;
	rightTicksPrev = *rightTicks;

	prevWheelComputeTime = micros();
}

unsigned long DeadReckonerPM::getChange(unsigned long current, unsigned long previous) {
	// Overflow has occured
	if (current < previous) {
		return UNSIGNED_LONG_MAX - previous + current;
	}
	// No overflow
	return current - previous;
}

void DeadReckonerPM::computePosition() {
	computeAngularVelocities();
	// Time elapsed after the previous position has been integrated.
	// change in time is defined as previous - current to prevent round off error.
	dt_integration = getChange(prevIntegrationTime, micros());

	float dt = dt_integration / 1000000.0; // convert to seconds

	// Dead reckoning equations

	float Vl = wl * radius;
	float Vr = wr * radius;
	float v = (Vr + Vl) / 2.0;
	w = (Vr - Vl) / length;
	// Uses 4th order Runge-Kutta to integrate numerically to find position.
	float xNext = xc + dt * v*(2 + cos(dt*w / 2))*cos(theta + dt * w / 2) / 3;
	float yNext = yc + dt * v*(2 + cos(dt*w / 2))*sin(theta + dt * w / 2) / 3;
	float thetaNext = theta + dt * w;

	xc = xNext;
	yc = yNext;
	theta = thetaNext;

	float toRPM = 30 / PI;
	float dist = sqrt(xc*xc + yc * yc);
	//Serial.print("\tdist: "); Serial.print(dist);
	//Serial.print("\twl: "); Serial.print(wl*toRPM, 5);
	//Serial.print("\twr: "); Serial.print(wr*toRPM, 5);
	//Serial.print("\tVl: "); Serial.print(Vl);
	//Serial.print("\tVr: "); Serial.print(Vr);
	//Serial.print("\tw: "); Serial.print(w, 5);
	//Serial.print("\tx: "); Serial.print(xc);
	//Serial.print("\ty: "); Serial.print(yc);
	//Serial.print("\ttheta: "); Serial.println(theta*RAD_TO_DEG);

	prevIntegrationTime = micros();
}

void DeadReckonerPM::reset() {
	xc = 0;
	yc = 0;
	theta = 0;
	leftTicksPrev = *leftTicks;
	rightTicksPrev = *rightTicks;
}
