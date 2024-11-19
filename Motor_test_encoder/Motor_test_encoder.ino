//This code rotates the output shaft from 0 to 360 degrees once.

//ESC wiring: Connect white signal wire on ESC to pin 9, black ground wire to Arduino ground.
//Encoder wiring: Channel A - Pin 2. Channel B - Pin 3. Power - 5V. GND - GND.

#include <Servo.h>

// Encoder pins
const int chA = 2; // Encoder Channel A connected to pin 2
const int chB = 3; // Encoder Channel B connected to pin 3

// Motor control
Servo esc_1;
const int motorPin = 9;

// Encoder tracking
volatile long encoderTicks = 0; // Track encoder ticks
const int ticksPerRevolution = 2632; // Ticks per revolution of the output shaft
const int gearBox = 28;
const float degreesPerTick = 360.0 / ticksPerRevolution / gearBox; // Degrees per encoder tick

void setup() {
  // Motor setup
  esc_1.attach(motorPin);

  // Encoder setup
  pinMode(chA, INPUT);
  pinMode(chB, INPUT);
  attachInterrupt(digitalPinToInterrupt(chA), readEncoder, CHANGE);

  // Initialize serial for plotting
  Serial.begin(9600);
  Serial.println("Starting motor rotation...");
}

void loop() {
  rotateMotorToDegrees(20); // Rotate motor to 360 degrees
  delay(2000);              // Pause for observation
  while (true) {
    // Stop further execution
  }
}

void rotateMotorToDegrees(float targetDegrees) {
  long targetTicks = targetDegrees / degreesPerTick; // Convert target degrees to encoder ticks
  setMotorPower(20); // Rotate forward
  while (encoderTicks < targetTicks) {
    float currentPosition = encoderTicks * degreesPerTick; // Convert ticks to degrees
    Serial.println(currentPosition); // Plot position in degrees
    delay(50); // Plot every 50ms
  }
  setMotorPower(0); // Stop the motor
}

void setMotorPower(int power) {
  power = constrain(power, -100, 100);
  int signalMin = 1050;
  int signalMax = 1950;
  int signalOutput = map(power, -100, 100, signalMin, signalMax);
  esc_1.writeMicroseconds(signalOutput);
}

void readEncoder() {
  int stateA = digitalRead(chA);
  int stateB = digitalRead(chB);
  if (stateA == stateB) {
    encoderTicks++; // Clockwise
  } else {
    encoderTicks--; // Counterclockwise
  }
}
