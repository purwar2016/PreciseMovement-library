

// Define your buttons and modes here
int MANUAL_MODE = 1;
int AUTO_MODE = 2;
int STOP_ROBOT = 0;
int LIFT = 3;

int mode = MANUAL_MODE; // Set default mode

void checkModes() {
  // CHECK FOR MODE CHANGES
  if (button == MANUAL_MODE) {
    mode = MANUAL_MODE;
    Serial.println("MANUAL MODE");
  }
  else if (button == AUTO_MODE) {
    mode = AUTO_MODE;
    Serial.println("AUTO MODE");
  }
  // Add more modes as needed.
}

void setup() {
  setupStuff(); // Demo code setup. Don't touch it.

  // Uncomment one at a time to test motors or encoders.
  // testMotors();
  // testEncoders();

  // Your setup below.
  
  Serial.println("SETUP COMPLETE: Path Follower");
}

void loop() {
  loopStuff(); // Demo code. Don't touch it!
  
  // CHECK FOR MODE CHANGES
  checkModes();

  // CHECK FOR BUTTONS THAT ARE USED IN BOTH AUTO AND MANUAL MODES
  if (button == STOP_ROBOT) {
    // If path following, stop following.
    premo.stop();
    Serial.println("STOP");
    // Stop the motors.
    stopMotors();
  }

  // HANDLE MANUAL MODE
  if (mode == MANUAL_MODE) {
    // Manual mode code here.

    // Follow path if you send a path.
    pathFollowing();

    // Steer the robot if you move the joystick.
    handleSteering();

    // More buttons here
    if (button == LIFT) {
      // Code for lift.
    }
    // Add buttons or sliders as needed
    // if (button == YOUR_BUTTON) {
    // }
  }

  // HANDLE AUTO MODE
  else if (mode == AUTO_MODE) {
    // Auto mode code here.

    // Add buttons or sliders as needed
    // if (button == YOUR_BUTTON) {
    // }
  }
}
