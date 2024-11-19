#include <Servo.h>

class Motor {
  private:
    Servo motor;
    int motorPin;
    int signalMin;
    int signalMax;

  public:
    Motor(int pin, int minSignal = 1050, int maxSignal = 1950) {
      motorPin = pin;
      signalMin = minSignal;
      signalMax = maxSignal;
    }

    void attach() {
      motor.attach(motorPin);
    }

    void setPower(int power) {
      power = constrain(power, -100, 100);
      int signalOutput = map(power, -100, 100, signalMin, signalMax);
      motor.writeMicroseconds(signalOutput);
    }
};

class Encoder {
  private:
    int chA;
    int chB;
    volatile long ticks;
    float degreesPerTick;

  public:
    Encoder(int channelA, int channelB, int ticksPerRevolution, int gearBox) 
      : chA(channelA), chB(channelB), ticks(0) {
      degreesPerTick = 360.0 / ticksPerRevolution / gearBox;
    }

    void setup() {
      pinMode(chA, INPUT);
      pinMode(chB, INPUT);
      attachInterrupt(digitalPinToInterrupt(chA), readEncoderISR, CHANGE);
    }

    long getTicks() const {
      return ticks;
    }

    float getDegrees() const {
      return ticks * degreesPerTick;
    }

    void resetTicks() {
      ticks = 0;
    }

    static Encoder* instance; // Pointer to the instance for ISR access

    static void readEncoderISR() {
      if (instance != nullptr) {
        int stateA = digitalRead(instance->chA);
        int stateB = digitalRead(instance->chB);
        if (stateA == stateB) {
          instance->ticks++; // Clockwise
        } else {
          instance->ticks--; // Counterclockwise
        }
      }
    }
};

class Ultrasonic {
  private:
    int trigPin;
    int echoPin;
    int distanceThreshold;
    unsigned long startTime;
    unsigned long timeThreshold;

  public:
    Ultrasonic(int trig, int echo, int dThresh = 20, int tThresh = 5000) {
      trigPin = trig;
      echoPin = echo;
      distanceThreshold = dThresh;
      pinMode(trigPin, OUTPUT);
      pinMode(echoPin, INPUT);

      startTime = 0;
      timeThreshold = tThresh;
    }

    long getDistance() {
      // Send a 10-microsecond pulse to the trigger pin
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);

      // Read the echo pin and calculate distance
      long duration = pulseIn(echoPin, HIGH);
      long distance = duration * 0.034 / 2; // Convert duration to distance in cm

      Serial.print(" Dist: " + String(distance));
      return distance;
    }

    bool isObjectDetected() {
      return getDistance() < distanceThreshold;
    }

    bool isSustainedDetection() {
      if (isObjectDetected()) {
        if (startTime == 0) {
          startTime = millis();
        } else if (millis() - startTime > timeThreshold) {
          return true;
        }
      } else {
        startTime = 0;
      }
      return false;
    }
};

Encoder* Encoder::instance = nullptr; // Initialize the static instance pointer


// Global variables for the sensors
Ultrasonic proteusSensor(6, 7);
Ultrasonic toteSensor(4, 5);
Motor motor(9);
Encoder encoder(2, 3, 2632, 28);

void setup() {
  motor.attach();

  encoder.setup();
  Encoder::instance = &encoder; // Assign the instance for ISR access

  Serial.begin(9600);
}

void loop() {
  // motor.setPower(10);
  rotateMotorToDegrees(0);
  Serial.print(" Encoder: " + String(encoder.getTicks()));

  Serial.println();
  delay(500); // Wait for a second before the next check
}

void rotateMotorToDegrees(float targetDegrees) {  
  int maxMotorSpeed = 15;
  float error = targetDegrees - encoder.getDegrees();
  float errorThresh = 0.2; 
  float kp = 1000000000; 

  while (abs(error) > errorThresh) {
    float currentPosition = encoder.getDegrees(); // Get current position in degrees
    error = targetDegrees - encoder.getDegrees();
    float motorSpeed = min(maxMotorSpeed, max(-maxMotorSpeed, kp * error));
    motor.setPower(100);
    Serial.println(" target: " + String(targetDegrees) + " curr : " + String(currentPosition) + " error: " + String(error) + " speed: " + String(motorSpeed)); // Plot position in degrees
    delay(50); // Plot every 50ms
  }
  motor.setPower(0); // Stop the motor
}