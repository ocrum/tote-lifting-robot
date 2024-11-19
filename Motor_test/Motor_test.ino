#include <Servo.h>

Servo esc_1;  // create servo object to control the PWM signal

void setup() {
  esc_1.attach(9);  // make sure to use a PWM capable pin
    Serial.begin(9600);

}

void loop() {  
  set_esc_power(esc_1, 10);  
  Serial.println("Running");
  delay(1000);
  // set_esc_power(esc_1, 0);
  // delay(5000) 
}

void set_esc_power (Servo esc, int power){
  power = constrain(power, -100, 100);
  int signal_min = 1050;
  int signal_max = 1950;
  int signal_output = map(power, -100, 100, signal_min, signal_max); //map(value, fromLow, fromHigh, toLow, toHigh)
  esc.writeMicroseconds(signal_output);
}