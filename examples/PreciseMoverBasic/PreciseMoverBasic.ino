#include <PreciseMover.h>

// REQUIRED means it requires adjustment.
// RECOMMENDED means adjustment is recommended for better performance.

// =======================================================================================================
// ========== ARDUINO PINS ===============================================================================
// =======================================================================================================

/* REQUIRED: All pins need checking  */

// Encoder pins
const int ENCODER_LEFT_PIN = 2;
const int ENCODER_RIGHT_PIN = 3;

// Motor one
const int ENA = 0;
const int IN1 = 0;
const int IN2 = 0;
// Motor two
const int ENB = 0;
const int IN3 = 0;
const int IN4 = 0;

// =======================================================================================================
// ========== ROBOT MEASUREMENTS =========================================================================
// =======================================================================================================

/* REQUIRED: Set LENGTH and RADIUS parameteres. */

// You can select any length unit as long as all three below have same untis.
const double RADIUS = 0; // Radius of the wheel in mm.
const double LENGTH = 0; // Distance from left to right wheel in mm.

// =======================================================================================================
// ========== PID PARAMETERS =============================================================================
// =======================================================================================================

/* RECOMMENDED: Tune the proportional and integral portions and to keep the derivative at zero. */

// PID parameters for pure pursuit (path following and going straight)
// These may require tuning for better results.
const double KP_FW = 100; // proportional
const double KI_FW = 0; // integral
const double KD_FW = 0; // derivative

// PID parameters for twisting
const double KP_TW = 10; // proportional
const double KI_TW = 0; // integral
const double KD_TW = 0; // derivative

// =======================================================================================================
// ========== FORWARD AND TWISTING MOTION PARAMETERS =====================================================
// =======================================================================================================

/* REQUIRED: Set PULSES_PER_REV and STOP_LENGTH. */
/* RECOMMENDED: Set MIN_MOTOR_SPEED */

// The number of pulses the encoder outputs for one wheel revolution.
const int PULSES_PER_REV = 144;

// To prevent overshoot, the motors will stop when it is STOP_LENGTH from the target distance.
const double STOP_LENGTH = 40; // mm

// The minimum PWM write value to move the motors when the robot is on the ground.
const int MIN_MOTOR_SPEED = 100; // PWM value from 0 to 255

// When the angle error reaches this threshold while twisting, the robot stops.
const int ANGLE_ERROR_THRESHOLD = 3; // 3 degrees

// This is the PWM target motor speed for the forward motion.
const int TARGET_FORWARD_SPEED = 200; // PWM value from 0 to 255

// This is the target angular velocity of the robot while twisting.
const int TARGET_TWIST_OMEGA = 15; // in RPM

// Look ahead parameter for pure pursuit algorithm.
// Don't worry about changing this unless you know what you're doing.
const double LOOK_AHEAD = 100; // in mm


// =======================================================================================================
// ========== VARIABLES ==================================================================================
// =======================================================================================================

// Keep track of encoder pulses
volatile unsigned long pulsesLeft = 0;
volatile unsigned long pulsesRight = 0;

// Previous pulse read times in microseconds
unsigned long prevLeftPulseReadTime = 0;
unsigned long prevRightPulseReadTime = 0;

// =======================================================================================================
// ========== PRECISEMOVER OBJECT ========================================================================
// =======================================================================================================

// Forward declarations needed for these methods.
void motorSetForward();
void motorSetCCW();
void motorSetCW();
void motorBrake();

// PreciseMover object
PreciseMover mover(
	&pulsesLeft,
	&pulsesRight,
	PULSES_PER_REV,
	ENA,
	ENB,
	LENGTH,
	RADIUS,
	MIN_MOTOR_SPEED,
	TARGET_FORWARD_SPEED,
	STOP_LENGTH,
	TARGET_TWIST_OMEGA,
	ANGLE_ERROR_THRESHOLD,
	&motorSetForward,
	&motorSetCCW,
	&motorSetCW,
	&motorBrake,
	KP_FW,
	KI_FW,
	KP_TW,
	KI_TW);

// =======================================================================================================
// ========== FUNCTIONS ==================================================================================
// =======================================================================================================

/* REQUIRED:
	Change the motorBrake, motorSetForward, motorSetCW, and motorSetCCW functions as necessary.
	Then, test them one by one and verify they work properly. */

// Configures the motor controller to stop the motors
void motorBrake() {
	digitalWrite(ENA, LOW);
	digitalWrite(ENB, LOW);
	digitalWrite(IN1, LOW);
	digitalWrite(IN2, LOW);
	digitalWrite(IN3, LOW);
	digitalWrite(IN4, LOW);
	digitalWrite(ENA, HIGH);
	digitalWrite(ENB, HIGH);
}

// Configures the motor controller to go forward.
void motorSetForward() {
	digitalWrite(IN1, HIGH);
	digitalWrite(IN2, LOW);
	digitalWrite(IN3, HIGH);
	digitalWrite(IN4, LOW);
}

// TODO: Configure dead reckoner!!!!!!!!!!!!!!!!!!
// Configures the motor controller to twist clockwise.
void motorSetCW() {
	digitalWrite(IN1, HIGH);
	digitalWrite(IN2, LOW);
	digitalWrite(IN3, LOW);
	digitalWrite(IN4, HIGH);
}

// Configures the motor controller to twist counter-clockwise.
void motorSetCCW() {
	digitalWrite(IN1, LOW);
	digitalWrite(IN2, HIGH);
	digitalWrite(IN3, HIGH);
	digitalWrite(IN4, LOW);
}

// Called whenever left pulse is detected
void pulseLeft() {
	// Only process the interrupt if it has been high for a minimum of 500 microseconds.
	if (digitalRead(ENCODER_LEFT_PIN) && micros() - prevLeftPulseReadTime > 250) {
		prevLeftPulseReadTime = micros();
		pulsesLeft++;
	}
}

// Called whenever right pulse is detected
void pulseRight() {
	// Only process the interrupt if it has been high for a minimum of 500 microseconds.
	if (digitalRead(ENCODER_RIGHT_PIN) && micros() - prevRightPulseReadTime > 250) {
		prevRightPulseReadTime = micros();
		pulsesRight++;
	}
}

// Attaches interrupts so that pulseLeft or pulseRight will be called whenever a pulse is detected
void attachInterrupts() {
	attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN), pulseLeft, HIGH);
	attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN), pulseRight, HIGH);
}

// =======================================================================================================
// ========== YOUR CODE HERE =============================================================================
// =======================================================================================================

/* RECOMMENDED:
	Use 115200 as the serial baud width.
	Make sure to set the baud rate accordingly on the Arduino serial monitor. */

void setup() {
	Serial.begin(115200);
	attachInterrupts();
	Serial.println(" ------------------ SETUP COMPLETE ------------------");
}

void loop() {
	// Move forward 1 meter
	mover.forward(1000);

	delay(2000);

	// Twist 90 degrees CCW
	mover.twist(90);

	delay(2000);
}
