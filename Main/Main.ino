#include <Servo.h>

/**
 * @brief Tracks motor rotation using an encoder
 * 
 * Converts encoder ticks to degrees, supporting precise 
 * motor position tracking with interrupt-based reading.
 */
class Encoder {
  private:
    int chA; // Encoder channel A pin
    int chB; // Encoder channel A pin
    volatile long ticks; // Cumulative encoder ticks
    float degreesPerTick; // Conversion factor from ticks to degrees

  public:
    /**
     * @brief Initialize encoder with specific parameters
     * 
     * @param channelA Pin for encoder channel A
     * @param channelB Pin for encoder channel B
     * @param ticksPerRevolution Total ticks per full rotation
     * @param gearBox Gear ratio multiplier
     */
    Encoder(int channelA, int channelB, int ticksPerRevolution, int gearBox) 
      : chA(channelA), chB(channelB), ticks(0) {
      degreesPerTick = 360.0 / ticksPerRevolution / gearBox;
    }

    /**
     * @brief Set up encoder pins and interrupt
     * 
     * Configures channel pins and attaches interrupt 
     * for real-time tick counting.
     */
    void setup() {
      pinMode(chA, INPUT);
      pinMode(chB, INPUT);
      Encoder::instance = this;
      attachInterrupt(digitalPinToInterrupt(chA), readEncoderISR, CHANGE);
    }

    /**
     * @brief Get the current number of encoder ticks
     * 
     * @return Current tick count since last reset
     */
    long getTicks() const {
      return ticks;
    }
    /**
     * @brief Convert encoder ticks to degrees of rotation
     * 
     * Calculates the total degrees rotated based on current 
     * tick count and the per-tick conversion factor.
     * 
     * @return Total degrees of rotation
     */
    float getDegrees() const {
      return ticks * degreesPerTick;
    }

    /**
     * @brief Reset the encoder tick count to zero
     * 
     * Resets the accumulated ticks, effectively setting 
     * the current position as the new zero point.
     */
    void resetTicks() {
      ticks = 0;
    }

    static Encoder* instance; // Pointer to the instance for ISR access

    /**
     * @brief Interrupt Service Routine for encoder reading
     * 
     * Updates tick count based on encoder channel states.
     * Determines rotation direction by comparing channel A and B states.
     * 
     * @note Requires static method to work with interrupt mechanism
     */
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

/**
 * @brief Manages motor control and positioning
 * 
 * Provides various motor control capabilities including simple power control,
 * Positional control including bang-bang style control and proportional control.
 */
class Motor {
  private:
    Servo motor;
    int dirPin1;
    int dirPin2;
    int pwmPin;
    int signalMin; // Minimum PWM signal value
    int signalMax; // Maximum PWM signal value
    float errorThresh; // Position error threshold for determining target position reached

  public:
    /**
     * @brief Construct a new Motor object
     * 
     * @param dirPin1 First direction control pin
     * @param dirPin2 Second direction control pin
     * @param pwmPin PWM pin for speed control
     * @param minSignal Minimum PWM signal (default 1050)
     * @param maxSignal Maximum PWM signal (default 1950)
     * @param errorThresh Position error threshold (default 0.2)
     */
    Motor(int dirPin1, int dirPin2, int pwmPin, int minSignal = 1050, int maxSignal = 1950, float errorThresh = 0.2f) {
      this->dirPin1 = dirPin1;
      this->dirPin2 = dirPin2;
      this->pwmPin = pwmPin;
      signalMin = minSignal;
      signalMax = maxSignal;
      this->errorThresh = errorThresh;
    }

    /**
     * @brief Initialize motor control pins
     * 
     * Sets direction and PWM pins to OUTPUT mode
     */
    void attach() {
      pinMode(dirPin1, OUTPUT);
      pinMode(dirPin2, OUTPUT);
      pinMode(pwmPin, OUTPUT);
    }

    /**
     * @brief Set motor power and direction
     * 
     * Controls motor movement by setting direction and speed:
     * - Positive power moves forward
     * - Negative power moves backward
     * - Zero power stops the motor
     * 
     * @param power Motor power (-100 to 100)
     */
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

    /**
     * @brief Control motor position using basic PID control
     * 
     * Moves motor towards target position using proportional control.
     * Supports two modes:
     * - PID mode: Adjusts speed proportional to position error
     * - Simple mode: Moves at fixed speed towards target
     * 
     * @param encoder Encoder to track motor position
     * @param targetDegrees Target position in degrees
     * @param mSpeed Maximum motor speed
     * @return bool True if target position is reached
     */
    bool controlMotor(Encoder& encoder, float targetDegrees, float mSpeed) {  
      int maxMotorSpeed = mSpeed;
      bool usePID = true;
      bool ret = false; // if reached target position

      float currentPosition = encoder.getDegrees(); // Get current position in degrees
      float error = targetDegrees - currentPosition;
      float kp = 20.0; 

      float motorSpeed = 0.0;

      if (abs(error) > errorThresh) {
        ret = false;
        if (usePID) {
          motorSpeed = constrain(kp * error, -maxMotorSpeed, maxMotorSpeed);
        } else {
          motorSpeed = (error > 0) ? maxMotorSpeed : -maxMotorSpeed;
        }
      } else {
        motorSpeed = 0;
        ret = true;
      }

      setPower(motorSpeed);
      return ret;
    }

    /**
     * @brief Check if motor has reached target position
     * 
     * @param encoder Encoder to track motor position
     * @param targetDegrees Target position in degrees
     * @return bool True if within error threshold
     */
    bool inPosition(Encoder& encoder, float targetDegrees) {
      float currentPosition = encoder.getDegrees(); // Get current position in degrees
      float error = targetDegrees - currentPosition;
      return (abs(error) <= errorThresh); // in position when within error threshold
    }
};

Encoder* Encoder::instance = nullptr; // Initialize the static instance pointer

/**
 * @brief Manages ultrasonic readings and recognizing object position states
 * 
 * Provides basic ultrasonic readings and also determining if an object is
 * is in place or clear.
 */
class Ultrasonic {
  private:
    int trigPin;
    int echoPin;
    int inPlaceThreshold;           // The maximum continuous distance to consider the object to be in place
    int clearThreshold;             // The minimum continuous distance to consider the space to be clear
    unsigned long startInPlaceTime; // When in place state first recognized 
    unsigned long startClearTime;   // When clear state first recognized
    unsigned long timeThreshold;    // How long an object is in place or clear to count as sustained  

  public:
    /**
     * @brief Constructs a new Ultrasonic object
     * 
     * @param trig Trigger pin number
     * @param echo Echo pin number
     * @param inPlaceThreshold The distance threshold to count as in place (cm)
     * @param clearThreshold The distance threshold to count as clear (cm)
     * @param tThresh The time threshold for if a state (in place/clear) is sustained (ms) (default 5000 ms or 5s)
     */
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

    /**
     * @brief Get the distance to an object for an ultrasonic sensor
     * 
     * Measures the time it takes for a sound pules to travel to an object and 
     * return, then calculates the distance in centimeters
     * 
     * @return The distance to the object in cm, or -1 if no echo is detected 
     */
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

      return distance;
    }

    /**
     * @brief Check if an object is in place
     * 
     * Uses the measured distance and determine if the object is in place using
     * the given threshold
     * 
     * @return True if the object is in place, false otherwise
     */
    bool isInPlace() {
      long distance = getDistance();
      return (distance != -1 && distance < inPlaceThreshold);
    }

     /**
     * @brief Check if the area is clear
     * 
     * Uses the measured distance and determine if the area is clear using
     * the given threshold
     * 
     * @return True if the area is clear, false otherwise
     */
    bool isClear() {
      long distance = getDistance();
      return (distance != -1 && distance > clearThreshold);
    }

    /**
     * @brief Checks if an object has been in place for a sustained duration
     * 
     * @return True if the object has been in place for the required duration,
     * false otherwise
     */
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

    /**
     * @brief Checks if the area has been clear of objects for a sustained
     * duration
     * 
     * @return True if the object has been in place for the required duration,
     * false otherwise
     */
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

/**
 * @brief Represents different operational states of the robot
 * 
 * Defines the high-level states the robot can operate in. Used for managing the
 * robot state
 */
enum SystemState{
  CALIBRATION,    /** State for calibrating start and stop points. */ 
  MANUAL_CONTROL, /** State for manually controlling the robot */
  AUTO_LOADING,   /** State for autonomously loading totes. */
  AUTO_UNLOADING, /** State for autonomously unloading totes. */
  IDLE            /** Idle state when the robot is not in operation. */
};

/**
 * @brief Manages the overall robot system
 * 
 * Represents the robot through its various sensors and actuators and manages
 * the states of the robot. Provides the system loop and testing methods.
 */
class RobotSystem {
  private:
    SystemState currState; 
    SystemState prevState;
    String userInput; // User input from the serial

    float targetPosition;   // Goal position of the motor (degrees)
    float conveyorPosition; // position of the conveyer belt (degrees)
    float proteusPosition;  // Position of proteus (degrees) 
    float idlePosition;     // Robot position when waiting (degrees) 
    float motorSpeed;       // Speed of the motor (% of max power)
    const float angleIncrement;  // How much to increment the angle in manual control 
    bool inPosition;        // If the robot is in position
    bool isRunning;         // If the robot is running

    Motor motor;
    Encoder encoder;
    Ultrasonic proteusSensor;
    Ultrasonic toteSensor;

    /**
     * @brief The goal sensing state for a given ultrasonic sensor
     */
    enum SensingState{
      IN_PLACE, 
      CLEAR,
      DONT_CARE /** Neither in place nor clear */
    };

    /**
     * @brief the type of autonomous sequence 
     */
    enum AutoType {
      LOADING, /** Loading totes from the conveyer belt to proteus */
      UNLOADING /** Unloading totes from proteus to the conveyer belt */
    };

    /**
     * @brief Represents a single step in the automated operation sequence. 
     */
    struct AutoStep {
      float position; /** Target position for the robot to move to (degrees) */
      SensingState proteusState; /** Desired state of the proteus sensor */
      SensingState conveyorState; /** Desired state of the conveyer sensor */
      String description; /** Text description of the action for this step */
    };

  public:
    /**
     * @brief Constructs a new RobotSystem object
     * 
     * Initializes the robot with default values for member variables, such as
     * setting the initial system state, motor speeds and pins # for components 
     */
    RobotSystem() :
      currState(IDLE),
      prevState(IDLE),
      userInput(""),

      targetPosition(0),
      conveyorPosition(75),
      proteusPosition(-75),
      motorSpeed(10),
      angleIncrement(3),
      inPosition(false),
      isRunning(false),

      motor(12, 13, 11),
      encoder(2, 3, 2632, 28),
      proteusSensor(6, 7, 7, 65),
      toteSensor(4, 5, 8, 40)
    {}

    /**
     * @brief Initializes the robot components
     * 
     * Attaches the motor and sets up the encoder
     */
    void setup() {
      motor.attach();
      encoder.setup();
      Encoder::instance = &encoder; // Assign the instance for ISR access

      displayInstructions();
    }

    /**
     * @brief Helper function to test if ultrasonics work
     */
    void testUltrasonics() {
      long p_dist = proteusSensor.getDistance();
      long t_dist = toteSensor.getDistance();
      Serial.print(" Proteus Sensor Distance: " + String(p_dist) + "cm");
      Serial.print(" Tote Sensor Distance: " + String(t_dist) + "cm");
      Serial.println();
    }

    /**
     * @brief Helper function to test if the encoder works (useful to test with
     * motor moving to see if the encoder values change)
     */
    void testEncoder() {
      Serial.println("Encoder Ticks: " + String(encoder.getTicks()));
    }

    /**
     * @brief Helper function to test for motor movement
     */
    void testMotor(int power) {
      motor.setPower(power);
    }

    /**
     * @brief Executes a single iteration of the robot's operational loop
     * 
     * During each loop, it processes user input, update the system state and
     * controls the motor accordingly. It is intended to be called repeatedly
     * within the main program loop to continuously manage the robot's behavior 
     */
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
      if (isRunning) {
        motor.controlMotor(encoder, targetPosition, motorSpeed);
      } else {
        motor.setPower(0);
      }
    }

  private:
    /**
     * @brief Displays the instructions for user input given the current state
     */
    void displayInstructions() {
      Serial.println("Current State: " + getStateName());

      String instructions;

      switch (currState) {
        case CALIBRATION:
          instructions = "'x': exit calibration | 'd': move forward | 'a': move backward | 'c': set conveyor position | 'p': set Proteus position | 'w' increase speed | 's' decrease speed";
          break;
        case MANUAL_CONTROL:
          instructions = "'x': exit manual control | 'd': move forward | 'a': move backward | 'w' increase speed | 's' decrease speed";
          break;
        case AUTO_LOADING:
          instructions = "'x': exit auto load Proteus | 'w' increase speed | 's' decrease speed";
          break;
        case AUTO_UNLOADING:
          instructions = "'x': exit auto unload Proteus | 'w' increase speed | 's' decrease speed";
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

    /**
     * @brief Method to get the string name of the current state
     * 
     * @return A string representing the name of the current state. Returns 
     * "unkown" if the state is unrecognized
     */
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

    /**
     * @brief Processes user inputs when calibrating 
     * 
     * Handles commands to:
     * - Change the target position (forwards: 'd'/backwards: 'a)
     * - Set the conveyer position 'c' or proteus position 'p'
     * - Increase 'w'/Decrease 's' the motor speed
     * - Exit the calibration state 'x'
     * 
     * Displays appropriate messages based on user actions 
     */
    void calibrate() {
      isRunning = true;
      if (userInput.length() > 0) {
        if (userInput == "d") {
          targetPosition += angleIncrement;
        } else if (userInput == "a") {
          targetPosition -= angleIncrement;
        } else if (userInput == "c") {
          conveyorPosition = targetPosition;
          idlePosition = (conveyorPosition + proteusPosition)/2;
          Serial.print("Conveyor position set to ");
          Serial.println(conveyorPosition);
        } else if (userInput == "p") {
          proteusPosition = targetPosition;
          idlePosition = (conveyorPosition + proteusPosition)/2;
          Serial.print("Proteus position set to ");
          Serial.println(proteusPosition);
        } else if (userInput == "w") { 
          motorSpeed = min(motorSpeed + 5, 100);
          Serial.print("Speed set to ");
          Serial.println(motorSpeed);
        } else if (userInput == "s") { 
          motorSpeed = max(motorSpeed - 5, 0);
          Serial.print("Speed set to ");
          Serial.println(motorSpeed);
        } else if (userInput == "x") {
          currState = IDLE;
        } else {
          Serial.println("Invalid input. Please try again.");
        }
      }
    }

    /**
     * @brief Processes user inputs during manual control 
     * 
     * Handles commands to:
     * - Change the target position (forwards: 'd'/backwards: 'a)
     * - Increase 'w'/Decrease 's' the motor speed
     * - Exit the calibration state 'x'
     * 
     * Displays appropriate messages based on user actions 
     */
    void manualControl() {
      isRunning = true;
      if (userInput.length() > 0) {
        if (userInput == "d") {
          targetPosition += angleIncrement;
        } else if (userInput == "a") {
          targetPosition -= angleIncrement;
        } else if (userInput == "w") { 
          motorSpeed = min(motorSpeed + 5, 100);
          Serial.print("Speed set to ");
          Serial.println(motorSpeed);
        } else if (userInput == "s") { 
          motorSpeed = max(motorSpeed - 5, 0);
          Serial.print("Speed set to ");
          Serial.println(motorSpeed);
        } else if (userInput == "x") {
          currState = IDLE;
        } else {
          Serial.println("Invalid input. Please try again.");
        }
      }
    }


    /**
     * @brief Processes user inputs and updates robot state variables when in
     * auto loading state
     */
    void autoLoad() {
        isRunning = true;
        executeAuto(LOADING);
    }

    /**
     * @brief Processes user inputs and updates robot state variables when in
     * auto unloading state
     */
    void autoUnload() {
      isRunning = true;
      executeAuto(UNLOADING);
    }

    /**
     * @brief Processes user inputs and updates robot state variables based off
     * of the given autonomous sequence 
     * 
     * This functions manages a sequence of steps (AutoSteps) based on the 
     * specified operation type. Additionally it processes user inputs for 
     * exiting the autonomous state and adjusting the motor speed. It progresses
     * through steps by moving into the given position if the sensor states are 
     * satisfied. It will only progress to the next state if the sensors and
     * the position is satisfied. 
     * 
     * @param type The type of automated operation (LOADING or UNLOADING)
     */
    void executeAuto(AutoType type) {
      static int stepIndex = 0; // Tracks current step #
      static AutoStep* steps = nullptr; // Pointer to the steps array
      static int totalSteps = 0; // Total steps in a single operation
      static bool operationInitialized = false; // Tracks if the operation steps are initialized

      if (userInput == "x") {
        currState = IDLE;
        stepIndex = 0;
        operationInitialized = false;
        targetPosition = idlePosition;
        return;
      } else if (userInput == "w") { 
          motorSpeed = min(motorSpeed + 5, 100);
          Serial.print("Speed set to "); Serial.println(motorSpeed);
        } else if (userInput == "s") { 
          motorSpeed = max(motorSpeed - 5, 0);
          Serial.print("Speed set to ");  Serial.println(motorSpeed);
      } 

      // initialize operation sequence if uninitialized 
      if (!operationInitialized) {
        if (type == LOADING) {
          static AutoStep loadingSteps[] = {
            // position.        proteus    conveyor.  description
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
            // position.        proteus    conveyor.  description
            { idlePosition,     DONT_CARE, DONT_CARE, "Ensure robot is in idle state and things are in place/clear"},
            { proteusPosition,  IN_PLACE,  CLEAR,     "Bring robot to proteus ensuring things are still in place"},
            { conveyorPosition, DONT_CARE, DONT_CARE, "Bring robot to conveyor ensuring conveyor is still clear"},
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
      bool isInPosition = motor.inPosition(encoder, targetPosition);

      // boolean logic to determine if the desired state and the actual state matches
      bool proteusSensorMatch = ((currentStep.proteusState == DONT_CARE) || 
                                 (currentStep.proteusState == CLEAR && proteusSensor.isClear()) ||
                                 (currentStep.proteusState == IN_PLACE && proteusSensor.isInPlace()));

      bool conveyorSensorMatch = ((currentStep.conveyorState == DONT_CARE) || 
                                  (currentStep.conveyorState == CLEAR && toteSensor.isClear()) ||
                                  (currentStep.conveyorState == IN_PLACE && toteSensor.isInPlace()));

      Serial.print("step: "); Serial.print(stepIndex);
      // Serial.print(" step: "); Serial.print(totalSteps);
      Serial.print(" targPos: "); Serial.print(targetPosition);
      Serial.print(" inPos: "); Serial.print(isInPosition);
      Serial.print(" pMatch: "); Serial.print(proteusSensorMatch);
      Serial.print(" tMatch: "); Serial.print(conveyorSensorMatch);
      Serial.print(" currPState: "); Serial.print(currentStep.proteusState);
      Serial.print(" currTState: "); Serial.print(currentStep.conveyorState);
      // Serial.print(" curr description: "); Serial.print(currentStep.description);
      Serial.println();

      if (isInPosition && proteusSensorMatch && conveyorSensorMatch) {
        // only progress to next step if everything matches
        stepIndex = (stepIndex + 1) % totalSteps;
      } else {
        if (!proteusSensorMatch || !conveyorSensorMatch) {
          // pause the system if things are not in place (or clear)
          isRunning = false;
        }
      }
    }

    /**
     * @brief Processes user inputs during manual control 
     * 
     * Handles commands to:
     * - Change the target position (forwards: 'd'/backwards: 'a)
     * - Increase 'w'/Decrease 's' the motor speed
     * - Exit the calibration state 'x'
     * 
     * Displays appropriate messages based on user actions 
     */
    void idle() {
      isRunning = false;
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
  // Uncomment the following to test the sensors and actuators: 
  // robotSystem.testUltrasonics();
  // robotSystem.testEncoder();
  // robotSystem.testMotor(10);
  robotSystem.run();
  delay(100);
}