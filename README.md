# Tote Lifting & Lowering Robot

A code repository for a robot that can lift and lower stacks of storage totes in a warehouse setting. A Tufts mechanical engineering senior capstone project for the client Amazon Robotics. Full report will be linked soon.

## Getting Started

### **Prerequisites**

- Hardware: Robot assembly as described in report
- Software: Arduino IDE (or similar)
- Libraries: The preinstalled default Servo.h

### **Installation & Setup**

1. Clone the repo:
    
    ```bash
    git clone https://github.com/ocrum/tote-lifting-robot.git
    ```
    
2. Upload `Main/Main.ino` to your Arduino Uno board
3. Follow the printed instructions

## Project Structure:

`Main/Main.ino`: Main code to run full robot system

`Monitor_ultrasonic/Monitor_ultrasonic.ino`: Code to test logic for monitoring ultrasonic states

`Motor_test/Motor_test.ino`: Simple motor code to run with [this motor controller](https://www.gobilda.com/1x15a-motor-controller/)

`Motor_test_encoder/Motor_test_encoder.ino`: Simple code to test the motor encoder 

`Motor_test_new_esc/Motor_test_new_esc.ino`: Simple code to test the motor with [this motor controller](https://www.amazon.com/dp/B0863HY8BT)

`Read_ultrasonic/Read_ultrasonic.ino`: Simple code to test ultrasonic readings