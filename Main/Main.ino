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

      motor.setPower(motorSpeed);
      DEBUG_PRINTLN(" target: " + String(targetDegrees) + " curr : " + String(currentPosition) + " error: " + String(error) + " speed: " + String(motorSpeed));
      return ret;
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
      long duration = pulseIn(echoPin, HIGH);
      long distance = duration * 0.034 / 2; // Convert duration to distance in cm

      DEBUG_PRINT(" Dist: " + String(distance));
      return distance;
    }

    bool isInPlace() {
      return getDistance() < inPlaceThreshold;
    }

    bool isClear() {
      return getDistance() < clearThreshold;
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



class System {
  private:
    SystemState currState;
    SystemState prevState;
    String userInput;

    float targetPosition; // Angle 
    float conveyerPosition;
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
      SensingState conveyerState;
      String description;
    };

  public:
    System() :
      currState(IDLE),
      prevState(IDLE),
      userInput(""),

      targetPosition(0),
      conveyerPosition(0),
      proteusPosition(0),
      angleIncrement(5),
      inPosition(false),

      motor(9),
      encoder(2, 3, 2632, 28),
      proteusSensor(6, 7, 10, 50), // TODO calibrate thresholds
      toteSensor(4, 5, 10, 50),
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
      inPosition = motor.rotateMotorToDegrees(encoder, targetPosition);
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
          conveyerPosition = targetPosition;
          idlePosition = (conveyerPosition + proteusPosition)/2;
          Serial.println("Conveyor position set to " + String(conveyerPosition));
        } else if (userInput == "p") {
          proteusPosition = targetPosition;
          idlePosition = (conveyerPosition + proteusPosition)/2;
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
            // position.        proteus    conveyer.  desciription
            { idlePosition,     DONT_CARE, DONT_CARE, "Ensure robot is in idle state and things are in place"},
            { conveyerPosition, IN_PLACE,  IN_PLACE,  "Bring robot to conveyer ensuring things are still in place"},
            { proteusPosition,  IN_PLACE,  DONT_CARE, "Bring robot to proteus ensuring proteus is still in place"},
            { proteusPosition,  CLEAR,     DONT_CARE, "Keep robot in place until proteus leaves"},
            { idlePosition,     DONT_CARE, DONT_CARE, "Bring robot back to idle"},
          };
          steps = loadingSteps;
          totalSteps = sizeof(loadingSteps) / sizeof(loadingSteps[0]);
        } else if (type = UNLOADING) {
          static AutoStep unloadingSteps[] = {
            // position.        proteus    conveyer.  desciription
            { idlePosition,     DONT_CARE, DONT_CARE, "Ensure robot is in idle state and things are in place/clear"},
            { proteusPosition,  IN_PLACE,  CLEAR,     "Bring robot to proteus ensuring things are still in place"},
            { conveyerPosition, DONT_CARE, DONT_CARE, "Bring robot to conveyer insureing conveyer is still clear"},
            { conveyerPosition, DONT_CARE, CLEAR,     "Keep robot in place until conveyer clear"},
            { idlePosition,     DONT_CARE, DONT_CARE, "Bring robot back to idle"},
          };
          steps = unloadingSteps;
          totalSteps = sizeof(loadingSteps) / sizeof(loadingSteps[0]);
        }
        operationInitialized = true;
        stepIndex = 0;
      }

      // TODO this is code that daniel wrote when i can't think

      AutoOperationStep& currentStep = steps[stepIndex];

      targetPosition = currentStep.position;

      // cehck if in position
      // check if sensors match

      // the idea: each step basically
      // target that position while things are in place/clear
      // if things are not inplace/clear then pause
      // move to next step when all things are in place/clear and at target


      // TODO look into dealing with ultrasonic noise (ignore really random jumps in data)
      // TODO consider looking into recovery behavior (if proteus moves)

      // TODO ensure that robot is in idle position before autoLoad?
      // make sure that it isn't loaded with a tote (current measurement)
      // or instead use user input somehow.
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