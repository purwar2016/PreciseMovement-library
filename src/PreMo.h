#ifndef PreMo_h
#define PreMo_h

#define TWIST_CCW 0
#define TWIST_CW 1
#define TWIST_MIN 2

#include "PID_v1.h"
#include "PurePursuit.h"
#include "DeadReckoner.h"
#include "EncoderManager.h"
#include "MotorManager.h"
#include "FunctionType.h"
#include <Arduino.h>

class CatmullRom;

class PreMo
{
public:
	static constexpr int PID_SAMPLE_TIME = 15; // ms
	static constexpr int PID_MOTOR_OUTPUT_RANGE = 100; // -100 to 100
	static constexpr int DEFAULT_PATH_FOLLOW_SPEED = 85; // percent

	PreMo(float radius, float length, double kp, double kd, float kpMotor, float kiMotor,
				MotorManager* motorManager, EncoderManager* encoderManager);
	void goToDelta(float deltaX, float deltaY);
	void goTo(float x, float y);
	void forward(float distance);
	void reverse(float distance);
	void twistBothMotors(bool twistBothMotors);
	void twist(float targetHeading, int twist=TWIST_MIN);
	void twistDelta(float angle);
	void continueTwist();
	void stop();
	void loop();
	bool isFollowingPath();
	void continuePathFollowing();
	void setPIDPathFollowing(float kp, float kd, float ki);
	void setPIDMotor(float kp, float kd, float ki);
	void startPathFollowing(float* pathX, float* pathY, int pathLength);
	void startPathFollowing(float* pathX, float* pathY, int pathLength, bool isForward, bool setLocation);
	void setPathFollowSpeed(int speedPercentage);
	double getX();
	void setX(double x);
	double getY();
	void setY(double y);
	double getGoalX();
	double getGoalY();
	double getHeading();
	float* getLocationData();
	double getOutput();
	void printPath();
	void reset();

private:
	static void transformCoordinate(float* x, float* y, float translateAngle, float translateX, float translateY);
	void moveMotors(int motorSpeed, double diff);

	bool _isFollowingPath;
	bool _moveReverse;

	EncoderManager* _encoderManager;

	static constexpr int _TEMP_PATH_LENGTH = 5;
	float _tempPathX[_TEMP_PATH_LENGTH];
	float _tempPathY[_TEMP_PATH_LENGTH];
	static constexpr float _END_STEP_ANGLE = 0.001 * PI/180; // rad
	void computeCurvePathPoint(float* x, float* y, float theta, float turningRadius, float transformAngle, float isLeftTurn);

	// Motors
	MotorManager* _motorManager;
	int _motorSpeed; // in percentage

	// Dead Reckoning
	DeadReckoner* _deadReckoner;
	// Pure Pursuit
	PurePursuit* _purePursuit;

	float _twistAngle;
	bool _isTwisting;
	float _targetHeading;
	bool _twistBothMotors;
	static constexpr float _TWIST_THRESHOLD_ANGLE = 2;

	// PID
	PID* _pid;
	double _input;
	double _output;
	double _setpoint;

	PID* _pidMotorLeft;
	double _inputMotorLeft;
	double _outputMotorLeft;
	double _setpointMotorLeft;

	PID* _pidMotorRight;
	double _inputMotorRight;
	double _outputMotorRight;
	double _setpointMotorRight;
};

#endif
