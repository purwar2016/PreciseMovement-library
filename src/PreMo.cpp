#include "PreMo.h"


/*
--------------------------------------------------------------------------------------------
PATH FOLLOWER CONTROL
--------------------------------------------------------------------------------------------
*/

PreMo::PreMo(float radius, float length, double kp, double kd, float kpMotor, float kiMotor,
				MotorManager* motorManager, EncoderManager* encoderManager)
{
	// Set variables
	_motorManager = motorManager;
	_encoderManager = encoderManager;
	_moveReverse = false;
	_motorSpeed = DEFAULT_PATH_FOLLOW_SPEED; // percent

	// Initialize DeadReckoner
	EncoderManager& em = (*_encoderManager);
	int ticksPerRev = em.getTicksPerRev();
	volatile unsigned long* leftPos = em.getLeftPosPointer();
	volatile unsigned long* rightPos = em.getRightPosPointer();
	_deadReckoner = new DeadReckoner(leftPos, rightPos, ticksPerRev, radius, length);

	// Initizlize PID
	_pid = new PID(&_input, &_output, &_setpoint, kp, 0, kd, DIRECT);
	(*_pid).SetSampleTime(PID_SAMPLE_TIME);
	(*_pid).SetOutputLimits(-PID_MOTOR_OUTPUT_RANGE, PID_MOTOR_OUTPUT_RANGE);
	(*_pid).SetMode(AUTOMATIC);
	_input = 0;
	_output = 0;
	_setpoint = 0;

	_pidMotorLeft = new PID(&_inputMotorLeft, &_outputMotorLeft, &_setpointMotorLeft, kpMotor, kiMotor, 0, DIRECT);
	(*_pidMotorLeft).SetSampleTime(PID_SAMPLE_TIME);
	(*_pidMotorLeft).SetOutputLimits(0, 100);
	(*_pidMotorLeft).SetMode(AUTOMATIC);
	_inputMotorLeft = 0;
	_outputMotorLeft = 0;
	_setpointMotorLeft = 0;

	_pidMotorRight = new PID(&_inputMotorRight, &_outputMotorRight, &_setpointMotorRight, kpMotor, kiMotor, 0, DIRECT);
	(*_pidMotorRight).SetSampleTime(PID_SAMPLE_TIME);
	(*_pidMotorRight).SetOutputLimits(0, 100);
	(*_pidMotorRight).SetMode(AUTOMATIC);
	_inputMotorRight = 0;
	_outputMotorRight = 0;
	_setpointMotorRight = 0;

	// Initizlie PurePursuit
	_purePursuit = new PurePursuit((*_deadReckoner).getXPointer(),
					(*_deadReckoner).getYPointer(), (*_deadReckoner).getHeadingPointer(), length);
}

void PreMo::goToDelta(float deltaX, float deltaY)
{
	DeadReckoner& dr = *_deadReckoner;
	double xr = dr.getX();
	double yr = dr.getY();
	float x = xr + deltaX;
	float y = yr + deltaY;
	goTo(x, y);
}

void PreMo::goTo(float x, float y)
{
	DeadReckoner& dr = *_deadReckoner;
	double xr = dr.getX();
	double yr = dr.getY();
	float theta = atan2(y - yr, x - xr);

	// Create the path to follow
	_tempPathX[0] = xr;
	_tempPathY[0] = yr;
	_tempPathX[1] = xr + cos(theta);
	_tempPathY[1] = yr + sin(theta);
	_tempPathX[2] = x - 2 * cos(theta);
	_tempPathY[2] = y - 2 * sin(theta);
	_tempPathX[3] = x - cos(theta);
	_tempPathY[3] = y - sin(theta);
	_tempPathX[4] = x;
	_tempPathY[4] = y;

	// Start following path
	startPathFollowing(_tempPathX, _tempPathY, 5, true, false);
}

void PreMo::forward(float distance)
{
	// Get the current pos and set the path array.
	double x = (*_deadReckoner).getX();
	double y = (*_deadReckoner).getY();
	double heading = (*_deadReckoner).getHeading();
	_tempPathX[0] = x;
	_tempPathY[0] = y;
	_tempPathX[1] = x + distance * 0.01 * cos(heading);
	_tempPathY[1] = y + distance * 0.01 * sin(heading);
	_tempPathX[2] = x + distance * 0.99 * cos(heading);
	_tempPathY[2] = y + distance * 0.99 * sin(heading);
	_tempPathX[3] = x + distance * cos(heading);
	_tempPathY[3] = y + distance * sin(heading);

	// Start path following
	startPathFollowing(_tempPathX, _tempPathY, 4, true, false);
}

void PreMo::reverse(float distance)
{
	// Get the current pos.
	double x = (*_deadReckoner).getX();
	double y = (*_deadReckoner).getY();
	double heading = (*_deadReckoner).getHeading();

	// Make the straight path
	_tempPathX[0] = x;
	_tempPathY[0] = y;
	_tempPathX[1] = x - distance * 0.01 * cos(heading);
	_tempPathY[1] = y - distance * 0.01 * sin(heading);
	_tempPathX[2] = x - distance * 0.99 * cos(heading);
	_tempPathY[2] = y - distance * 0.99 * sin(heading);
	_tempPathX[3] = x - distance * cos(heading);
	_tempPathY[3] = y - distance * sin(heading);

	// Start path following
	startPathFollowing(_tempPathX, _tempPathY, 4, false, false);
}

void PreMo::twistBothMotors(bool twistBothMotors)
{
	_twistBothMotors = twistBothMotors;
}

void PreMo::twist(float targetHeading, int direction=TWIST_MIN)
{
	targetHeading = targetHeading - 360 * floor(targetHeading / 360);

	double heading = (*_deadReckoner).getHeading()*RAD_TO_DEG;
	// Serial.print("heading: "); Serial.print(heading);
	heading = heading - 360 * floor(heading / 360);

	// float delta = heading - targetHeading;
	float delta1 = targetHeading - heading;
	float delta2 = fabs(360 - fabs(delta1));
	float delta;
	float deltaCW;
	float deltaCCW;

	if (delta1 > 0)
	{
		deltaCCW = delta1;
		deltaCW = -delta2;
	}
	else
	{
		deltaCCW = delta2;
		deltaCW = delta1;
	}
	
	if (direction == TWIST_CCW)
	{
		delta = deltaCCW;
	}
	else if (direction == TWIST_CW)
	{
		delta = deltaCW;
	}
	else if (direction == TWIST_MIN)
	{
		delta = (fabs(deltaCCW) < fabs(deltaCW)) ? (deltaCCW) : (deltaCW);
	}

	// Serial.print("\theading2: "); Serial.print(heading);
	// Serial.print("\ttheading: "); Serial.print(targetHeading);
	// Serial.print("\tdelta1: "); Serial.print(delta1);
	// Serial.print("\tdelta2: "); Serial.print(delta2);
	// Serial.print("\tdelta: "); Serial.println(delta);

	twistDelta(delta);
}

void PreMo::twistDelta(float angle)
{
	// Serial.print("angle: "); Serial.println(angle);
	_setpointMotorLeft = 40; // RPM
	_setpointMotorRight = 40;
	_inputMotorLeft = 0;
	_inputMotorRight = 0;

	DeadReckoner& dr = *_deadReckoner;
	float heading = dr.getHeading()*RAD_TO_DEG;
	_targetHeading = heading + angle;
	_twistAngle = angle;
	_isTwisting = true;

	// Serial.print("target: "); Serial.println(_targetHeading);
}

void PreMo::continueTwist()
{
	DeadReckoner& dr = *_deadReckoner;
	MotorManager& mm = *_motorManager;

	double heading = dr.getHeading()*RAD_TO_DEG;
	if ((_twistAngle > 0 && _targetHeading - heading > _TWIST_THRESHOLD_ANGLE) || (_twistAngle < 0 && heading - _targetHeading > _TWIST_THRESHOLD_ANGLE))
	{
		_inputMotorLeft = fabs(dr.getWl()*DeadReckoner::RAD_PER_SEC_TO_RPM);
		_inputMotorRight = fabs(dr.getWr()*DeadReckoner::RAD_PER_SEC_TO_RPM);
		if (_twistAngle > 0)
		{
			if (_twistBothMotors)
			{
				mm.leftReverse(_outputMotorLeft);
				dr.setLeftDirection(WHEEL_REVERSE);
			}
			mm.rightForward(_outputMotorRight);
			dr.setRightDirection(WHEEL_FORWARD);
		}
		else
		{
			if (_twistBothMotors)
			{
				mm.rightReverse(_outputMotorRight);
				dr.setRightDirection(WHEEL_REVERSE);
			}
			mm.leftForward(_outputMotorLeft);
			dr.setLeftDirection(WHEEL_FORWARD);
		}
		(*_pidMotorLeft).Compute();
		(*_pidMotorRight).Compute();

		// Serial.print(heading);
		// Serial.print(", "); Serial.print(heading - _targetHeading);
		// Serial.print(", ");  Serial.print(_inputMotorLeft);
		// Serial.print(", "); Serial.println(_inputMotorRight);
	}
	else
	{
		_isTwisting = false;

		// Stop the motors.
		stop();
	}
}

void PreMo::transformCoordinate(float* x, float* y, float transformAngle, float translateX, float translateY)
{
	float tempX = *x;
	float tempY = *y;
	*x = translateX + tempX * cos(transformAngle) + tempY * sin(transformAngle);
	*y = translateY - tempX * sin(transformAngle) + tempY * cos(transformAngle);
}

void PreMo::computeCurvePathPoint(float* x, float* y, float theta, float turningRadius, float transformAngle, float isLeftTurn)
{
	// Get the current position.
	double xPos = (*_deadReckoner).getX();
	double yPos = (*_deadReckoner).getY();

	theta = isLeftTurn ? theta : -theta;
	float xip = turningRadius * (cos(theta) - 1);
	float yip = turningRadius * sin(theta);

	transformCoordinate(&xip, &yip, transformAngle, xPos, yPos);

	// Serial.print(xPos); Serial.print("\t");
	// Serial.print(yPos); Serial.print("\t");
	// Serial.print(theta); Serial.print("\t");
	// Serial.print(xip); Serial.print("\t");
	// Serial.println(yip);

	*x = xip;
	*y = yip;
}

void PreMo::setPIDPathFollowing(float kp, float kd, float ki)
{
	(*_pid).SetTunings(kp, ki, kd);
}

void PreMo::setPIDMotor(float kp, float kd, float ki)
{
	(*_pidMotorLeft).SetTunings(kp, ki, kd);
	(*_pidMotorRight).SetTunings(kp, ki, kd);
}

void PreMo::startPathFollowing(float* pathX, float* pathY, int pathLength)
{
	startPathFollowing(pathX, pathY, pathLength, true, true);
}

void PreMo::startPathFollowing(float* pathX, float* pathY, int pathLength, bool isForward, bool setLocation)
{
	// Set starting position to first path location point with robot pointing towards the next point.
	double x0 = pathX[0];
	double y0 = pathY[0];
	double x1 = pathX[1];
	double y1 = pathY[1];

	double heading = atan2(y1 - y0, x1 - x0);

	// Serial.print("x0: "); Serial.print(x0);
	// Serial.print("\ty0: "); Serial.print(y0);
	// Serial.print("\tx1: "); Serial.print(x1);
	// Serial.print("\ty1: "); Serial.print(y1);
	// Serial.print("\theading: "); Serial.println(heading);

	// Set the location to the initial coordinates.
	DeadReckoner& dr = (*_deadReckoner);
	// Set the heading to the angle of the vector from the first to the second point.
	heading = isForward ? heading : heading + PI;

	if (setLocation)
	{
		// Set the starting location as the first path point.
		dr.setX(x0);
		dr.setY(y0);

		// Set the heading such that the robot points to the initial direction that the path will follow.
		dr.setHeading(heading);
	}

	(*_purePursuit).setPath(pathX, pathY, pathLength);
	// (*_purePursuit).printPath();

	_moveReverse = !isForward;
	_isFollowingPath = true;
	(*_purePursuit).start();
	
	// TODO Remove
	// double x = dr.getX();
	// double y = dr.getY();
	// double t = dr.getHeading();
	// Serial.println("DEAD RECKONER SET");
	// Serial.print("x: "); Serial.print(x);
	// Serial.print(", y: "); Serial.print(y);
	// Serial.print(", t: "); Serial.println(t);
}

void PreMo::setPathFollowSpeed(int speedPercentage)
{
	_motorSpeed = speedPercentage;
}

void PreMo::stop()
{
	_isFollowingPath = false;
	_isTwisting = false;
	(*_motorManager).stop();
}

void PreMo::continuePathFollowing()
{
	// Pure pursuit
	(*_purePursuit).compute();

	// Compute PID
	_input = (*_purePursuit).getCurvature();
	(*_pid).Compute();

	// Move the robot
	// Serial.print("output: "); Serial.println(_output);
	moveMotors(_motorSpeed, _output);

	if ((*_purePursuit).checkStop())
	{
		_isFollowingPath = false;
		stop();
	}
}

void PreMo::loop()
{
	// Dead reckoning
	(*_deadReckoner).computePosition();

	if (_isFollowingPath)
	{
		continuePathFollowing();
	}
	else if (_isTwisting)
	{
		continueTwist();
	}
}

bool PreMo::isFollowingPath()
{
	return _isFollowingPath;
}

void PreMo::moveMotors(int motorSpeed, double diff)
{
	int speedLeft, speedRight;

	// Set speed negation
	// pos diff -> turn left
	// neg diff -> turn right
	if (diff > 0) {
		speedLeft = map(diff, 100, 0, 0, motorSpeed);
		speedRight = motorSpeed;
	}
	else {
		diff = -diff;
		speedLeft = motorSpeed;
		speedRight = map(diff, 100, 0, 0, motorSpeed);
	}
	if (!_moveReverse)
	{
		(*_deadReckoner).setLeftDirection(WHEEL_FORWARD);
		(*_deadReckoner).setRightDirection(WHEEL_FORWARD);
		(*_motorManager).leftForward(speedLeft);
		(*_motorManager).rightForward(speedRight);
	}
	else
	{
		(*_deadReckoner).setLeftDirection(WHEEL_REVERSE);
		(*_deadReckoner).setRightDirection(WHEEL_REVERSE);
		(*_motorManager).leftReverse(speedLeft);
		(*_motorManager).rightReverse(speedRight);
	}
}

double PreMo::getX()
{
	return (*_deadReckoner).getX();
}

void PreMo::setX(double x)
{
	(*_deadReckoner).setX(x);
}

double PreMo::getY()
{
	return (*_deadReckoner).getY();
}

void PreMo::setY(double y)
{
	(*_deadReckoner).setY(y);
}

double PreMo::getGoalX()
{
	return (*_purePursuit).getGoalX();
}
double PreMo::getGoalY()
{
	return (*_purePursuit).getGoalY();
}
double PreMo::getHeading()
{
	return (*_deadReckoner).getHeading();
}

float* PreMo::getLocationData()
{
	static float data[5];
	PurePursuit pp = (*_purePursuit);
	DeadReckoner dr = (*_deadReckoner);
	data[0] = dr.getX();
	data[1] = dr.getY();
	data[2] = dr.getHeading();
	data[3] = pp.getGoalX();
	data[4] = pp.getGoalY();
	return data;
}

double PreMo::getOutput()
{
	return _output;
}

void PreMo::printPath()
{
	(*_purePursuit).printPath();
}

void PreMo::reset()
{
	(*_deadReckoner).reset();
	stop();
}
