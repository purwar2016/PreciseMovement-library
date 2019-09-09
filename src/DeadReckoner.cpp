#include "DeadReckoner.h"
#include <Arduino.h>

DeadReckoner::DeadReckoner() { }

DeadReckoner::DeadReckoner(volatile unsigned long *left, volatile unsigned long *right,
							int tpr, double r, double l, unsigned long ci=50) {
	Init(left, right, tpr, r, l, ci);
}

void DeadReckoner::Init(volatile unsigned long *left, volatile unsigned long *right,
							int tpr, double r, double l, unsigned long ci=50) {
	leftTicks = left;
	rightTicks = right;
	ticksPerRev = tpr;
	radius = r;
	length = l;
	toRadPerSec = 1000000.0 * TWO_PI / ticksPerRev;
	positionComputeInterval = ci*1000; // millisecond to microseconds
	prevIntegrationTime = micros();
	prevWheelComputeTime = micros();
	leftTicksPrev = (*leftTicks);
	rightTicksPrev = (*rightTicks);

	wl = 0;
	wr = 0;
	xc = 0;
	yc = 0;
	heading = 0;
	w = 0;
	prevIntegrationTime = micros();
	prevWheelComputeTime = micros();

}

void DeadReckoner::setX(double x) {
	xc = x;
}

void DeadReckoner::setY(double y) {
	yc = y;
}

void DeadReckoner::setHeading(double h) {
	heading = h;
}

double DeadReckoner::getX() {
	return xc;
}

double DeadReckoner::getY() {
	return yc;
}

double* DeadReckoner::getXPointer() {
	return &xc;
}

double* DeadReckoner::getYPointer() {
	return &yc;
}

double DeadReckoner::getWl() {
	// TODO: Check this
	return wl;
}

double DeadReckoner::getWr() {
	// TODO: Check this
	return wr;
}

double DeadReckoner::getW() {
	return w;
}

double DeadReckoner::getHeading() {
	return heading;
}

double* DeadReckoner::getHeadingPointer() {
	return &heading;
}

void DeadReckoner::setLeftDirection(int direction) {
	leftOmegaDirection = direction;
}

void DeadReckoner::setRightDirection(int direction) {
	rightOmegaDirection = direction;
}

int DeadReckoner::getLeftOmegaDirection() {
	return leftOmegaDirection;
}

int DeadReckoner::getRightOmegaDirection() {
	return rightOmegaDirection;
}

void DeadReckoner::computeAngularVelocities() {
	// Time elapsed after computing the angular velocity previously.
	// change in time is defined as previous - current to prevent round off error.
	dt_omega = getChange(micros(), prevWheelComputeTime); // in milliseconds

	double changeLeftTicks = getChange(*leftTicks, leftTicksPrev);
	double changeRightTicks = getChange(*rightTicks, rightTicksPrev);

	wl = leftOmegaDirection * changeLeftTicks / dt_omega * toRadPerSec;
	wr = rightOmegaDirection * changeRightTicks / dt_omega * toRadPerSec;

	unsigned long lt = (*leftTicks);
	unsigned long rt = (*rightTicks);
	// Serial.print("\twl: "); Serial.print(wl*toRPM);
	// Serial.print("\twr: "); Serial.print(wr*toRPM);
	// Serial.print("\tdt: "); Serial.print(dt_omega);
	// Serial.print("\tlt: "); Serial.print(lt);
	// Serial.print("\trt: "); Serial.print((rt));
	// Serial.print("\tDlt: "); Serial.print(changeLeftTicks);
	// Serial.print("\tDrt: "); Serial.println(changeRightTicks);

	leftTicksPrev = *leftTicks;
	rightTicksPrev = *rightTicks;

	prevWheelComputeTime = micros();
}

void DeadReckoner::computePosition() {
	if (micros() - prevIntegrationTime > positionComputeInterval) {
		computeAngularVelocities();
		// Time elapsed after the previous position has been integrated.
		// change in time is defined as previous - current to prevent round off error.
		dt_integration = getChange(micros(), prevIntegrationTime);

		float dt = dt_integration / 1000000.0; // convert to seconds

		// Dead reckoning equations

		float Vl = wl * radius;
		float Vr = wr * radius;
		float v = (Vr + Vl) / 2.0;
		w = (Vr - Vl) / length;
		// Uses 4th order Runge-Kutta to integrate numerically to find position.
		float xNext = xc + dt * v*(2 + cos(dt*w / 2))*cos(heading + dt * w / 2) / 3;
		float yNext = yc + dt * v*(2 + cos(dt*w / 2))*sin(heading + dt * w / 2) / 3;
		float headingNext = heading + dt * w;

		xc = xNext;
		yc = yNext;
		heading = headingNext;

		// Debug Info
		// float toRPM = 30 / PI;
		// float dist = sqrt(xc*xc + yc*yc);
		// Serial.print("dist: "); Serial.print(dist);
		// Serial.print("\tx: "); Serial.print(xc);
		// Serial.print("\ty: "); Serial.print(yc);
		// Serial.print("\theading: "); Serial.print(heading*RAD_TO_DEG);
		// Serial.print("\twl: "); Serial.print(wl*toRPM, 5);
		// Serial.print("\twr: "); Serial.print(wr*toRPM, 5);
		// Serial.print("\tVl: "); Serial.print(Vl);
		// Serial.print("\tVr: "); Serial.print(Vr);
		// Serial.print("\tw: "); Serial.println(w);

		prevIntegrationTime = micros();
	}
}

unsigned long DeadReckoner::getChange(unsigned long current, unsigned long previous) {
	// Overflow has occured
	if (current < previous) {
		return UNSIGNED_LONG_MAX - previous + current;
	}
	// No overflow
	return current - previous;
}

void DeadReckoner::reset() {
	xc = 0;
	yc = 0;
	wl = 0;
	wr = 0;
	w = 0;
	heading = 0;
}
