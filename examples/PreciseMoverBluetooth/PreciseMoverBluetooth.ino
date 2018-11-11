#include <PreciseMover.h>
#include <SoftwareSerial.h>
#include <ArduinoBlue.h>

// Forward declarations needed for these methods.
void motorSetForward();
void motorSetCCW();
void motorSetCW();
void motorBrake();

// =======================================================================================================
// ========== ARDUINO PINS ===============================================================================
// =======================================================================================================

// Encoder pins
const int ENCODER_LEFT_PIN = 2;
const int ENCODER_RIGHT_PIN = 3;

// motor one
const int ENA = 5;
const int IN1 = 4;
const int IN2 = 6;
// motor two
const int ENB = 11;
const int IN3 = 10;
const int IN4 = 9;

// HM10 bluetooth module pins
const int BLUETOOTH_TX = 8;
const int BLUETOOTH_RX = 7;

// =======================================================================================================
// ========== ROBOT MEASUREMENTS =========================================================================
// =======================================================================================================

// You can select any length unit as long as all three below have same untis.
const double RADIUS = 29; // mm
const double LENGTH = 102; // mm

// =======================================================================================================
// ========== PID PARAMETERS =============================================================================
// =======================================================================================================

// PID parameters for pure pursuit (path following and going straight)
const double KP_FW = 100;
const double KI_FW = 0;
const double KD_FW = 0;

// PID parameters for twisting	
const int PID_TW_MIN = 100;
const int PID_TW_MAX = 255;
const double KP_TW = 10;
const double KI_TW = 0;
const double KD_TW = 0;

// =======================================================================================================
// ========== FORWARD AND TWISTING MOTION PARAMETERS =====================================================
// =======================================================================================================

// The motors will stop when it is this length from the target.
// This prevents overshoot.
const double STOP_LENGTH = 40; // mm

// Acceptable angle error after twisting. 
const int ANGLE_ERROR_THRESHOLD = 3; // 3 degrees

// target PWM motor speed forward movement.
// This should be a slow enough for stability
const int TARGET_FORWARD_SPEED = 200; // PWM value 0 to 255

// target robot angular velocity during twisting motion.
const int TARGET_TWIST_OMEGA = 15; // in RPM

// Number of pulses of the encoder per revolution of wheel.
const int PULSES_PER_REV = 144;

// Look ahead parameter for pure pursuit algorithm.
// Don't worry about changing this unless you know what you're doing.
const double LOOK_AHEAD = 100;

// =======================================================================================================
// ========== VARIABLES ==================================================================================
// =======================================================================================================

// Keep track of encoder pulses
volatile unsigned long pulsesLeft = 0;
volatile unsigned long pulsesRight = 0;

// Previous pulse read times in microseconds
unsigned long prevLeftPulseReadTime = 0;
unsigned long prevRightPulseReadTime = 0;

const int MIN_MOTOR_SPEED = 100;

// Constructors
SoftwareSerial bluetoothSerial(BLUETOOTH_TX, BLUETOOTH_RX);
ArduinoBlue bluetoothController(bluetoothSerial);
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

void setup() {
	Serial.begin(115200);
	attachInterrupts();
	Serial.println(" ------------------ SETUP COMPLETE ------------------");
}

void loop() {
	// Move forward 1 meter
	mover.forward(1000);

	delay(2000);

	// Twist 90 degrees clockwise
	mover.twist(90);

	delay(2000);
}
