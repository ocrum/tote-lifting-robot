#include <Servo.h>

// Uncomment the line below to enable debugging
#define DEBUG

#ifdef DEBUG
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif

class Motor {
  private:
    Servo motor;
    int motorPin;
    int signalMin;
    int signalMax;
    int errorThresh;

  public:
    Motor(int pin, int minSignal = 1050, int maxSignal = 1950, int errorThresh = 0.2) {
      motorPin = pin;
      signalMin = minSignal;
      signalMax = maxSignal;
      errorThresh = errorThresh;
    }

    void attach() {
      motor.attach(motorPin);
    }

    void setPower(int power) {
      power = constrain(power, -100, 100);
      int signalOutput = map(power, -100, 100, signalMin, signalMax);
      motor.writeMicroseconds(signalOutput);
    }

    void rotateMotorToDegrees(Encoder& encoder, float targetDegrees) {  
      int maxMotorSpeed = 15;
      bool usePID = false;

      float error = targetDegrees - encoder.getDegrees();
      float kp = maxMotorSpeed; 

      // TODO consider making this a function that doesn't have an internal loop
      while (abs(error) > errorThresh) {
        float currentPosition = encoder.getDegrees(); // Get current position in degrees
        error = targetDegrees - currentPosition;

        float motorSpeed = 0

        if (usePID) {
          float motorSpeed = min(maxMotorSpeed, max(-maxMotorSpeed, kp * error));
        } else {
          if (erorr > 0) {
            motorSpeed = maxMotorSpeed;
          } else {
            motorSpeed = -maxMotorSpeed;
          }
        }

        motor.setPower(motorSpeed);
        DEBUG_PRINTLN(" target: " + String(targetDegrees) + " curr : " + String(currentPosition) + " error: " + String(error) + " speed: " + String(motorSpeed));
        delay(50); 
      }
      motor.setPower(0); // Stop the motor
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

Encoder* Encoder::instance = nullptr; // Initialize the static instance pointer

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

      DEBUG_PRINT(" Dist: " + String(distance));
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

enum SystemState{
  CALIBRATION, // Calibrate start and stop points
  MANUAL_CONTROL, // Controlling the robot manually using
  AUTO_LOADING,
  AUTO_UNLOADING,
  IDLE
};

class System {
  private:
    SystemState currState;
    String userInput;
    float targetPosition; // Angle 

    Motor motor;
    Encoder encoder;
    Ultrasonic proteusSensor;
    Ultrasonic toteSensor;

  public:
    System()
      : currentState(IDLE),
        motor(9),
        encoder(2, 3, 2632, 28),
        proteusSensor(6, 7),
        toteSensor(4, 5)
        userInput("")
        targetPosition(0) {}

    void setup() {
      motor.attach();
      encoder.setup();
      Encoder::instance = &encoder; // Assign the instance for ISR access
    }

    void run() {
      // Get input
      if (Serial.available() > 0) {
        userInput = Serial.readStringUntil('\n');
      }

      // Handle input and update state
      switch (currentState) {
        case CALIBRATION:
          calibrate();
          break;
        case MANUAL_CONTROL:
          manualControl();
          break;
        case AUTO_LOADING:
          autoLoad();
          break;
        case AUTO_UNLOADING:
          autoUnload();
          break;
        case IDLE:
          idle();
          break;
      }

      // Render state
      motor.rotateMotorToDegrees(encoder, targetPosition);
    }

  private:
    String getStateName() const {
      switch (currentState) {
        case CALIBRATION: return "Calibration";
        case MANUAL_CONTROL: return "Manual";
        case AUTO_LOADING: return "Load Proteus";
        case AUTO_UNLOADING: return "Unload Proteus";
        case IDLE: return "IDLE";
        default: return "UNKNOWN";
      }
    }

    void calibrate() {
      Serial.print("'D': move forward 'A': move backward 'C': set conveyer position 'P': set Proteus position")

      swithch (userInput) {
        case "A":
        case "D":
        case "C":
        case "P":
      }
    }

    void manualControl() {

    }

    void autoLoad() {

    }

    void autoUnload() {

    }

    void idle() {
      
    }

    
}

System system;

void setup() {
  Serial.begin(9600);
  system.setup();
}

void loop() {
  system.run();
  delay(1000);
}