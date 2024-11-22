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
      Encoder::instance = this;
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

class Motor {
  private:
    Servo motor;
    int dirPin1;
    int dirPin2;
    int pwmPin;
    int signalMin;
    int signalMax;
    int errorThresh;

  public:
    Motor(int dirPin1, int dirPin2, int pwmPin, int minSignal = 1050, int maxSignal = 1950, int errorThresh = 0.2f) {
      this->dirPin1 = dirPin1;
      this->dirPin2 = dirPin2;
      this->pwmPin = pwmPin;
      signalMin = minSignal;
      signalMax = maxSignal;
      this->errorThresh = errorThresh;
    }

    void attach() {
      pinMode(dirPin1, OUTPUT);
      pinMode(dirPin2, OUTPUT);
      pinMode(pwmPin, OUTPUT);
    }

    void setPower(int power) {
      // Constrain power to range -100 to 100
      power = constrain(power, -100, 100);

      if (power > 0) {
        // Forward direction
        digitalWrite(dirPin1, HIGH);
        digitalWrite(dirPin2, LOW);
        analogWrite(pwmPin, map(power, 0, 100, 0, 255)); // Scale 0-100 to 0-255
      } else if (power < 0) {
        // Reverse direction
        digitalWrite(dirPin1, LOW);
        digitalWrite(dirPin2, HIGH);
        analogWrite(pwmPin, map(-power, 0, 100, 0, 255)); // Scale 0-100 to 0-255
      } else {
        // Stop the motor
        digitalWrite(dirPin1, LOW);
        digitalWrite(dirPin2, LOW);
        analogWrite(pwmPin, 0);
      }
    }

    bool controlMotor(Encoder& encoder, float targetDegrees) {  
      int maxMotorSpeed = 15;
      bool usePID = false;
      bool ret = false; // if reached target position

      float currentPosition = encoder.getDegrees(); // Get current position in degrees
      float error = targetDegrees - currentPosition;
      float kp = maxMotorSpeed; 

      float motorSpeed = 0.0;

      if (abs(error) > errorThresh) {
        ret = false;
        if (usePID) {
          motorSpeed = constrain(kp * error, -maxMotorSpeed, maxMotorSpeed);
        } else {
          motorSpeed = (error > 0) ? maxMotorSpeed : -maxMotorSpeed;
        }
      } else {
        ret = true;
      }

      setPower(motorSpeed);
      DEBUG_PRINTLN(" target: " + String(targetDegrees) + " curr : " + String(currentPosition) + " error: " + String(error) + " speed: " + String(motorSpeed));
      return ret;
    }

    bool inPosition(Encoder& encoder, float targetDegrees) {
      float currentPosition = encoder.getDegrees(); // Get current position in degrees
      float error = targetDegrees - currentPosition;
      return (abs(error) <= errorThresh);
    }
};

Encoder* Encoder::instance = nullptr; // Initialize the static instance pointer

class Ultrasonic {
  private:
    int trigPin;
    int echoPin;
    int inPlaceThreshold; // The maximum continous distance to cosndier the object to be in place
    int clearThreshold;   // The minimum continous distance to consider the space to be clear
    unsigned long startInPlaceTime;
    unsigned long startClearTime;
    unsigned long timeThreshold;

  public:
    Ultrasonic(int trig, int echo, int inPlaceThreshold, int clearThreshold, int tThresh = 5000)
      : trigPin(trig),
        echoPin(echo),
        inPlaceThreshold(inPlaceThreshold),
        clearThreshold(clearThreshold),
        startInPlaceTime(0),
        startClearTime(0),
        timeThreshold(tThresh) {
      pinMode(trigPin, OUTPUT);
      pinMode(echoPin, INPUT);
    }

    long getDistance() {
      // Send a 10-microsecond pulse to the trigger pin
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);

      // Read the echo pin and calculate distance
      long duration = pulseIn(echoPin, HIGH, 30000);
      if (duration == 0) return -1;
      long distance = duration * 0.034 / 2; // Convert duration to distance in cm

      DEBUG_PRINT(" Dist: " + String(distance));
      return distance;
    }

    bool isInPlace() {
      long distance = getDistance();
      return (distance != -1 && distance < inPlaceThreshold);
    }

    bool isClear() {
      long distance = getDistance();
      return (distance != -1 && distance > clearThreshold);
    }

    bool isSustainedInPlace() {
      if (isInPlace()) {
        if (startInPlaceTime == 0) {
          startInPlaceTime = millis();
        } else if (millis() - startInPlaceTime > timeThreshold) {
          return true;
        }
      } else {
        startInPlaceTime = 0;
      }
      return false;
    }

    bool isSustainedClear() {
      if (isClear()) {
        if (startClearTime == 0) {
          startClearTime = millis();
        } else if (millis() - startClearTime > timeThreshold) {
          return true;
        }
      } else {
        startClearTime = 0;
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



class RobotSystem {
  private:
    SystemState currState;
    SystemState prevState;
    String userInput;

    float targetPosition; // Angle 
    float conveyorPosition;
    float proteusPosition;
    float idlePosition;
    const float angleIncrement;
    bool inPosition;

    Motor motor;
    Encoder encoder;
    Ultrasonic proteusSensor;
    Ultrasonic toteSensor;

    enum SensingState{
      IN_PLACE,
      CLEAR,
      DONT_CARE
    };

    enum AutoType {
      LOADING,
      UNLOADING
    };

    struct AutoStep {
      float position;
      SensingState proteusState;
      SensingState conveyorState;
      String description;
    };

  public:
    RobotSystem() :
      currState(IDLE),
      prevState(IDLE),
      userInput(""),

      targetPosition(0),
      conveyorPosition(0),
      proteusPosition(0),
      angleIncrement(5),
      inPosition(false),

      motor(2, 3, 9),
      encoder(2, 3, 2632, 28),
      proteusSensor(6, 7, 10, 50), // TODO calibrate thresholds
      toteSensor(4, 5, 10, 50)
    {}

    void setup() {
      motor.attach();
      encoder.setup();
      Encoder::instance = &encoder; // Assign the instance for ISR access

      displayInstructions();
    }

    void run() {
      if (currState != prevState) {
        displayInstructions();
        prevState = currState;
      }

      // Get input
      if (Serial.available() > 0) {
        userInput = Serial.readStringUntil('\n');
        userInput.trim();
      } else {
        userInput = "";
      }

      // Handle input and update state
      switch (currState) {
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
      motor.controlMotor(encoder, targetPosition);
    }

  private:
    void displayInstructions() {
      Serial.println("Current State: " + getStateName());

      String instructions;

      switch (currState) {
        case CALIBRATION:
          instructions = "'x': exit calibration | 'd': move forward | 'a': move backward | 'c': set conveyor position | 'p': set Proteus position";
          break;
        case MANUAL_CONTROL:
          instructions = "'x': exit manual control | 'd': move forward | 'a': move backward";
          break;
        case AUTO_LOADING:
          instructions = "'x': exit auto load Proteus";
          break;
        case AUTO_UNLOADING:
          instructions = "'x': exit auto unload Proteus";
          break;
        case IDLE:
          instructions = "'c': calibrate | 'm': manual control | 'l': auto load Proteus | 'u': auto unload Proteus";
          break;
        default:
          instructions = "No instructions available.";
          break;
      }

      Serial.println(instructions);
      Serial.print("> ");
    }

    String getStateName() const {
      switch (currState) {
        case CALIBRATION:    return "Calibration";
        case MANUAL_CONTROL: return "Manual";
        case AUTO_LOADING:   return "Load Proteus";
        case AUTO_UNLOADING: return "Unload Proteus";
        case IDLE:           return "Idle";
        default:             return "Unknown";
      }
    }

    void calibrate() {
      if (userInput.length() > 0) {
        if (userInput == "d") {
          targetPosition += angleIncrement;
        } else if (userInput == "a") {
          targetPosition -= angleIncrement;
        } else if (userInput == "c") {
          conveyorPosition = targetPosition;
          idlePosition = (conveyorPosition + proteusPosition)/2;
          Serial.println("Conveyor position set to " + String(conveyorPosition));
        } else if (userInput == "p") {
          proteusPosition = targetPosition;
          idlePosition = (conveyorPosition + proteusPosition)/2;
          Serial.println("Proteus position set to " + String(proteusPosition));
        } else if (userInput == "x") {
          currState = IDLE;
        } else {
          Serial.println("Invalid input. Please try again.");
        }
      }

    }

    void manualControl() {
      if (userInput.length() > 0) {
        if (userInput == "d") {
          targetPosition += angleIncrement;
        } else if (userInput == "a") {
          targetPosition -= angleIncrement;
        } else if (userInput == "x") {
          currState = IDLE;
        } else {
          Serial.println("Invalid input. Please try again.");
        }
      }
    }

    void autoLoad() {
        executeAuto(LOADING);
    }

    void autoUnload() {
      executeAuto(UNLOADING);
    }

    void executeAuto(AutoType type) {
      static int stepIndex = 0; // How is this stored between loop calls
      static AutoStep* steps = nullptr; // Pointer to the steps array
      static int totalSteps = 0;
      static bool operationInitialized = false;

      if (userInput == "x") {
        currState = IDLE;
        stepIndex = 0;
        operationInitialized = false;
        return;
      }

      if (!operationInitialized) {
        if (type == LOADING) {
          static AutoStep loadingSteps[] = {
            // position.        proteus    conveyor.  desciription
            { idlePosition,     DONT_CARE, DONT_CARE, "Ensure robot is in idle state and things are in place"},
            { conveyorPosition, IN_PLACE,  IN_PLACE,  "Bring robot to conveyor ensuring things are still in place"},
            { proteusPosition,  IN_PLACE,  DONT_CARE, "Bring robot to proteus ensuring proteus is still in place"},
            { proteusPosition,  CLEAR,     DONT_CARE, "Keep robot in place until proteus leaves"},
            { idlePosition,     DONT_CARE, DONT_CARE, "Bring robot back to idle"},
          };
          steps = loadingSteps;
          totalSteps = sizeof(loadingSteps) / sizeof(loadingSteps[0]);
        } else if (type = UNLOADING) {
          static AutoStep unloadingSteps[] = {
            // position.        proteus    conveyor.  desciription
            { idlePosition,     DONT_CARE, DONT_CARE, "Ensure robot is in idle state and things are in place/clear"},
            { proteusPosition,  IN_PLACE,  CLEAR,     "Bring robot to proteus ensuring things are still in place"},
            { conveyorPosition, DONT_CARE, DONT_CARE, "Bring robot to conveyor insureing conveyor is still clear"},
            { conveyorPosition, DONT_CARE, CLEAR,     "Keep robot in place until conveyor clear"},
            { idlePosition,     DONT_CARE, DONT_CARE, "Bring robot back to idle"},
          };
          steps = unloadingSteps;
          totalSteps = sizeof(unloadingSteps) / sizeof(unloadingSteps[0]);
        }
        operationInitialized = true;
        stepIndex = 0;
      }

      AutoStep& currentStep = steps[stepIndex];

      targetPosition = currentStep.position;

      bool proteusSensorMatch = ((currentStep.proteusState != DONT_CARE) || 
                                 (currentStep.proteusState == CLEAR && proteusSensor.isClear()) ||
                                 (currentStep.proteusState == IN_PLACE && proteusSensor.isInPlace()));

      bool conveyorSensorMatch = ((currentStep.conveyorState != DONT_CARE) || 
                                  (currentStep.conveyorState == CLEAR && toteSensor.isClear()) ||
                                  (currentStep.conveyorState == IN_PLACE && toteSensor.isInPlace()));

      if (motor.inPosition(encoder, targetPosition) && proteusSensorMatch && conveyorSensorMatch) {
        stepIndex = (stepIndex + 1) % totalSteps;
      }
    }

    void idle() {
      if (userInput.length() > 0) {
        if (userInput == "c") {
          currState = CALIBRATION;
        } else if (userInput == "m") {
          currState = MANUAL_CONTROL;
        } else if (userInput == "l") {
          currState = AUTO_LOADING;
        } else if (userInput == "u") {
          currState = AUTO_UNLOADING;
        } else {
          Serial.println("Invalid input. Please try again.");
        }
      }
    }  
};

RobotSystem robotSystem;

void setup() {
  Serial.begin(9600);
  robotSystem.setup();
}

void loop() {
  robotSystem.run();
  delay(1000);
}