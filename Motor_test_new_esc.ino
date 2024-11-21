// Define control pins
int DirPin1 = 2;  // Direction pin 1
int DirPin2 = 3;  // Direction pin 2
int PWMPin = 9;   // PWM pin

void setup() {
  pinMode(DirPin1, OUTPUT);
  pinMode(DirPin2, OUTPUT);
  pinMode(PWMPin, OUTPUT);

  // Start serial communication
  Serial.begin(9600);
  Serial.println("Use arrow keys to control the motor:");
  Serial.println("W/UP: Forward");
  Serial.println("S/DOWN: Reverse");
  Serial.println("X: Stop");
}

void loop() {
  // Check if data is available from the Serial Monitor
  if (Serial.available() > 0) {
    char command = Serial.read(); // Read the input character

    switch (command) {
      case 'w': // Move forward
      case 'W': // 'W' key
        driveMotor(30);  // Adjust speed as needed
        Serial.println("Motor moving forward.");
        break;
      case 's': // Move backward
      case 'S': // 'S' key
        driveMotor(-30); // Adjust speed as needed
        Serial.println("Motor moving backward.");
        break;
      case 'x': // Stop the motor
      case 'X': // 'X' key
        driveMotor(0);
        Serial.println("Motor stopped.");
        break;
      default:
        Serial.println("Invalid input. Use W/UP, S/DOWN, or X.");
    }
  }
}

// Function to drive the motor
void driveMotor(int power) {
  // Constrain power to range -100 to 100
  power = constrain(power, -100, 100);

  if (power > 0) {
    // Forward direction
    digitalWrite(DirPin1, HIGH);
    digitalWrite(DirPin2, LOW);
    analogWrite(PWMPin, map(power, 0, 100, 0, 255)); // Scale 0-100 to 0-255
  } else if (power < 0) {
    // Reverse direction
    digitalWrite(DirPin1, LOW);
    digitalWrite(DirPin2, HIGH);
    analogWrite(PWMPin, map(-power, 0, 100, 0, 255)); // Scale 0-100 to 0-255
  } else {
    // Stop the motor
    digitalWrite(DirPin1, LOW);
    digitalWrite(DirPin2, LOW);
    analogWrite(PWMPin, 0);
  }
}
