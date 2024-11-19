// Define pins for Proteus Sensor
const int TrigProteusPin = 4;
const int EchoProteusPin = 5;

// Define pins for Tote Sensor
const int TrigTotePin = 6;
const int EchoTotePin = 7;

// Variables to monitor presence
bool proteus_presence = false;
bool tote_presence = false;

// Timing variables
unsigned long proteusStartTime = 0;
unsigned long toteStartTime = 0;

// Distance threshold (in cm)
const int distanceThreshold = 20;

// Time threshold (in milliseconds)
const unsigned long presenceDurationThreshold = 5000;

void setup() {
  // Set up pins for Proteus Sensor
  pinMode(TrigProteusPin, OUTPUT);
  pinMode(EchoProteusPin, INPUT);

  // Set up pins for Tote Sensor
  pinMode(TrigTotePin, OUTPUT);
  pinMode(EchoTotePin, INPUT);

  // Initialize serial communication
  Serial.begin(9600);
}

void loop() {
  // Check presence for both sensors
  check_proteus_presence();
  check_tote_presence();

  // Update timing for presence
  if (proteus_presence && tote_presence) {
    if (proteusStartTime == 0 || toteStartTime == 0) {
      proteusStartTime = millis();
      toteStartTime = millis();
    } else if ((millis() - proteusStartTime > presenceDurationThreshold) &&
               (millis() - toteStartTime > presenceDurationThreshold)) {
      Serial.println("Ready to move totes");
    }
  } else {
    // Reset timing if any presence condition is not met
    proteusStartTime = 0;
    toteStartTime = 0;
  }

  // Delay for 1 second before checking again
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

// Function to check Proteus presence
void check_proteus_presence() {
  long distance = getDistance(TrigProteusPin, EchoProteusPin);
  proteus_presence = (distance < distanceThreshold);
}

// Function to check Tote presence
void check_tote_presence() {
  long distance = getDistance(TrigTotePin, EchoTotePin);
  tote_presence = (distance < distanceThreshold);
}
