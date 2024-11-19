// Define pins for Sensor 1
const int trigPin1 = 4;
const int echoPin1 = 5;

// Define pins for Sensor 2
const int trigPin2 = 6;
const int echoPin2 = 7;

void setup() {
  // Set up pins for Sensor 1
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);

  // Set up pins for Sensor 2
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);

  // Initialize serial communication
  Serial.begin(9600);
}

void loop() {
  // Get distance readings from both sensors
  long distance1 = getDistance(trigPin1, echoPin1);
  long distance2 = getDistance(trigPin2, echoPin2);

  // Print the readings to the Serial Monitor
  Serial.print("Sensor 1 Distance: ");
  Serial.print(distance1);
  Serial.print(" cm, ");
  Serial.print("Sensor 2 Distance: ");
  Serial.print(distance2);
  Serial.println(" cm");

  // Wait for 1 second before the next reading
  delay(1000);
}

// Function to calculate distance from an HC-SR04 sensor
long getDistance(int trigPin, int echoPin) {
  // Send a 10-microsecond pulse to the trigger pin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the echo pin and calculate distance
  long duration = pulseIn(echoPin, HIGH);
  long distance = duration * 0.034 / 2; // Convert duration to distance in cm
  return distance;
}
