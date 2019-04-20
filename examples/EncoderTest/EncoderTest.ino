/*
Tests the encoder
*/

const int ENCODER_LEFT_PIN = 2;
const int ENCODER_RIGHT_PIN = 3;

volatile unsigned long leftTicks = 0, rightTicks = 0;


void pulseLeft() { leftTicks++; }
void pulseRight() { rightTicks++; }

void attachInterrupts() {
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN), pulseLeft, HIGH);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN), pulseRight, HIGH);
}

void setup() {
  // put your setup code here, to run once:
  attachInterrupts();
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("left: "); Serial.print(leftTicks);
  Serial.print("\tright: "); Serial.println(rightTicks);
}
