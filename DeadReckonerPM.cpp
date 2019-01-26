#include <Arduino.h>
#include "PreciseMover.h"
#include "DeadReckonerPM.h"
#include "PidPM.h"


PreciseMover::PreciseMover(
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
        int newTargetTwistOmega,
        int newAngleErrorThreshold,
        functiontype newMotorSetForward,
        functiontype newMotorSetCCW,
        functiontype newMotorSetCW,
        functiontype newMotorBrake,
        double newKpFW,
        double newKiFW,
        double newKpTW,
        double newKiTW) {

    // Set the variables
    pulsesLeft = newPulsesLeft;
    pulsesRight = newPulsesRight;
    PULSES_PER_REV = newPulsesPerRev;
    ENA = newEnA;
    ENB = newEnB;
    LENGTH = newLength;
    RADIUS = newRadius;
    MIN_MOTOR_SPEED = newMinMotorSpeed;
    TARGET_FORWARD_SPEED = newTargetForwardSpeed;
    STOP_LENGTH = newStopLength;
    TARGET_TWIST_MOTOR_OMEGA = newTargetTwistOmega;
    ANGLE_ERROR_THRESHOLD = newAngleErrorThreshold*DEG_TO_RAD;
    motorSetForward = newMotorSetForward;
    motorSetCCW = newMotorSetCCW;
    motorSetCW = newMotorSetCW;
    motorBrake = newMotorBrake;
    kpFW = newKpFW;
    kiFW = newKiFW;
    kpTW = newKpTW;
    kiTW = newKiTW;

    // Set the PIDs
    pidFW.construct(&inputFW, &outputFW, &setpointFW, kpFW, kiFW, kdFW, DIRECT);
    pidFW.SetSampleTime(PID_SAMPLE_TIME);
    pidFW.SetOutputLimits(PID_MIN_FW, PID_MAX_FW);
    pidFW.SetMode(AUTOMATIC);

	pidLeftMotorTW.construct(&inputLeftTW, &outputLeftTW, &setpointLeftTW, kpTW, kiTW, kdTW, DIRECT);
	pidLeftMotorTW.SetSampleTime(PID_SAMPLE_TIME);
	pidLeftMotorTW.SetOutputLimits(PID_MIN_TW, PID_MAX_TW);
	pidLeftMotorTW.SetMode(AUTOMATIC);

	pidRightMotorTW.construct(&inputRightTW, &outputRightTW, &setpointRightTW, kpTW, kiTW, kdTW, DIRECT);
	pidRightMotorTW.SetSampleTime(PID_SAMPLE_TIME);
	pidRightMotorTW.SetOutputLimits(PID_MIN_TW, PID_MAX_TW);
	pidRightMotorTW.SetMode(AUTOMATIC);

    // Set DeadReckoner
    deadReckoner.construct(pulsesLeft, pulsesRight, PULSES_PER_REV, RADIUS, LENGTH);
}

// abs method for double values. Built-in abs is only for integers.
double absDouble(double n) {
    return n < 0 ? -n : n;
}

void PreciseMover::moveWheels(int motorSpeed, double diff) {
    int speedLeft, speedRight;

    // Set speed negation
    // pos diff -> turn left
    // neg diff -> turn right
    if (diff > 0) {
        speedLeft = map(diff, PID_MAX_FW, 0, 0, motorSpeed);
        speedRight = motorSpeed;
    }
    else {
        diff = -diff;
        speedLeft = motorSpeed;
        speedRight = map(diff, PID_MAX_FW, 0, 0, motorSpeed);
    }
    analogWrite(ENA, speedLeft);
    analogWrite(ENB, speedRight);
}

void PreciseMover::doDeadReckoning() {
    if (millis() - prevPositionComputeTime > POSITION_COMPUTE_INTERVAL) {
        // Computes the new angular velocities and uses that to compute the new position.
        // The accuracy of the position estimate will increase with smaller time interval until a certain point.
        deadReckoner.computePosition();
        prevPositionComputeTime = millis();
    }
}

// Get the goal point for the Pure Pursuit algorithm
double PreciseMover::getGoalPointX(double xPos, double yPos, double xTarget) {
    // compute the magnitude of the vector from the origin to the goal point
    double inside = 1 - yPos * yPos / (LOOK_AHEAD*LOOK_AHEAD);
    if (inside >= -1 && inside <= 1) {
        double theta = acos(inside);
        return xPos + LOOK_AHEAD * cos(theta);
    }
    return xPos;
}

void PreciseMover::forward(double distance) {
    //Serial.print("distance: "); Serial.print(distance);
    // Assume starting poing is the origin so that the ultimate target point will
    // be (distance, 0)
    deadReckoner.reset();
    deadReckoner.setLeftOmegaDirection(WHEEL_FORWARD);
    deadReckoner.setRightOmegaDirection(WHEEL_FORWARD);
    //deadReckoner.setX(0);
    //deadReckoner.setY(0);
    //deadReckoner.setTheta(0);

    //Serial.print("startX: "); Serial.print(deadReckoner.getX());
    //Serial.print("\tstartY: "); Serial.println(deadReckoner.getY());

    // Set motors to move forward
    motorSetForward();


    double xGoal = 0;
    double yGoal = 0;
    double xPos, yPos, phi, xGoalRelRobot, curvature;

    double distanceAway = 9999999999;
    while (distanceAway > STOP_LENGTH) {
        doDeadReckoning();
        if (millis() - prevCorrectionTime > CORRECTION_TIME_INTERVAL_FW) {
            // 1. Find the current location of the vehicle.
            xPos = deadReckoner.getX();
            yPos = deadReckoner.getY();
            phi = deadReckoner.getTheta() - PI / 2;

            // 2. Find the path point closest to the vehicle.
            // Not necessary for this implementation.

            // 3. Find the goal point
            // the y goal point is not needed since the path is straight with yi in the path set to zero.
            xGoal = getGoalPointX(xPos, yPos, distance);
            yGoal = 0;

            // 4. Transform the goal point to vehicle coordinates.
            // First translate it such that the origin is the robot.
            xGoal -= xPos;
            yGoal -= yPos;
            // Rotate it so that the top of the robot is the y-axis
            xGoalRelRobot = xGoal * cos(phi) + yGoal * sin(phi);

            // 5. Calculate the curvature
            curvature = 2 * xGoalRelRobot / LOOK_AHEAD;

            // Input to the PID controller is the curvature
            inputFW = curvature;

            // Approximate distance from the target x to the current robot location.
            distanceAway = abs(xPos - distance);

            prevCorrectionTime = millis();
            
			/*Serial.print("DistanceAway: "); Serial.print(distanceAway);
            Serial.print("\txr: "); Serial.print(xPos);
            Serial.print("\tyr: "); Serial.print(yPos);
            Serial.print("\tphi: "); Serial.print(phi*RAD_TO_DEG);
            Serial.print("\txGoal: "); Serial.println(xGoal);*/
        }
        pidFW.Compute();
        moveWheels(TARGET_FORWARD_SPEED, outputFW);
    }

    // Stop motor once the target is close enough
    motorBrake();
}

void PreciseMover::twist(double angle) {
    Serial.println("twist called");
    // Convert angle to radians
    angle *= DEG_TO_RAD;
    // Serial.print("angle: "); Serial.print(angle);
    deadReckoner.reset();
    double startAngle = deadReckoner.getTheta();
    double currentAngle = startAngle;

    //setpoint_TW = angle;
	setpointLeftTW = TARGET_TWIST_MOTOR_OMEGA;
	setpointRightTW = TARGET_TWIST_MOTOR_OMEGA;

    double angleError = angle - (currentAngle - startAngle); // (target angle) - (change in angle)

    // abs function on Arduino is only defined for integers! Using it will yield a bug.
    while (absDouble(angleError) > ANGLE_ERROR_THRESHOLD) {
        doDeadReckoning();
        currentAngle = deadReckoner.getTheta();
        if (millis() - prevCorrectionTime > CORRECTION_TIME_INTERVAL_TW) {
            // Set PID input
            angleError = angle - (currentAngle - startAngle); // (target angle) - (change in angle)

			inputLeftTW = absDouble(deadReckoner.getWl() * 30.0 / PI);
			inputRightTW = absDouble(deadReckoner.getWr() * 30.0 / PI);

            if (angleError > 0) {
                motorSetCCW();
                deadReckoner.setLeftOmegaDirection(WHEEL_REVERSE);
                deadReckoner.setRightOmegaDirection(WHEEL_FORWARD);
            }
            else {
                motorSetCW();
                deadReckoner.setLeftOmegaDirection(WHEEL_FORWARD);
                deadReckoner.setRightOmegaDirection(WHEEL_REVERSE);
            }
            prevCorrectionTime = millis();

            /* Serial.print("\tcurrentAngle: "); Serial.print(currentAngle*RAD_TO_DEG);
            Serial.print("\tangleError: "); Serial.print(angleError*RAD_TO_DEG);
            Serial.print("\tthresholdAngle: "); Serial.print(ANGLE_ERROR_THRESHOLD*RAD_TO_DEG);
            Serial.print("\toutputTW: "); Serial.print(outputTW); */
        }
		pidLeftMotorTW.Compute();
		pidRightMotorTW.Compute();
		
		/* Serial.print("\toutputLeft: "); Serial.print(outputLeftTW);
		Serial.print("\toutputRight: "); Serial.println(outputRightTW); */
		analogWrite(ENA, outputLeftTW);
		analogWrite(ENB, outputRightTW);
    }

    //angleError = angle - currentAngle - startAngle;
    motorBrake();
}

void PreciseMover::debug() {
    doDeadReckoning();
    double x = deadReckoner.getX();
    double y = deadReckoner.getY();
    Serial.print("x: "); Serial.print(x);
    Serial.print("\ty: "); Serial.println(y);
    //motorSetForward();
    //motorSetForward();
    motorBrake();
}

// Prints the angular velocity of the whole robot (not individual wheels) for the Serial plotter.
void PreciseMover::printPlot(int actual, int target) {
    Serial.print(actual);
    Serial.print(" ");
    Serial.print(target);
}

void PreciseMover::tuneTwistPID() {
    // Rotate CCW for TUNE_DELAY ms and CW for TUNE_DELAY ms 
	// TODO: implement this or get rid of it. I think get rid of it since new algo for twisting. New algo is to PID individual motor left and right.
    /*bool isCCW = false;
    unsigned long prevDirectionChange = millis();
    deadReckoner.reset();
    while (true) {
        if (isCCW) {
            motorSetCW();
            deadReckoner.setLeftOmegaDirection(WHEEL_FORWARD);
            deadReckoner.setRightOmegaDirection(WHEEL_REVERSE);
            isCCW = false;
        }
        else {
            motorSetCCW();
            deadReckoner.setLeftOmegaDirection(WHEEL_REVERSE);
            deadReckoner.setRightOmegaDirection(WHEEL_FORWARD);
            isCCW = true;
        }
        while (millis() - prevDirectionChange < tuneDelay) {
            inputTW = deadReckoner.getW()*PI / 30;
            //printPlot(inputTW, TARGET_TWIST_OMEGA);
            pidTW.Compute();
            moveWheels(outputTW, 0);
        }
        prevDirectionChange = millis();
    }*/
}

void PreciseMover::setTuneDelay(int delay) {
    tuneDelay = delay;
}
