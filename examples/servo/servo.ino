/* This example project can be used to control a servo motor attached to PWM pin D3 
 * Angle of servo motor can be controlled through serial monitor of Arduino IDE (115200 bauds with no line ending)
 * Or with a potentiometer connected to pin A3
 */


#include <Servo.h>

Servo servo1;
long angle;
int potPin = A3;
int potVal;
int currentPotVal;

void setup() {
  servo1.attach(3); 
  Serial.begin(115200);
}
 
void loop() {
  potVal = analogRead(potPin);
  if (Serial.available()) {
    angle = Serial.parseInt();
    Serial.print("Set servo angle to: "); // [0 ; 170]
    Serial.println(angle);
    servo1.write(angle);
  }
  if (abs(currentPotVal-potVal) > 10)
  {
    angle = map(potVal, 0, 1023, 0, 170);
    Serial.print("Potentiometer val: ");
    Serial.println(potVal);
    servo1.write(angle);
    Serial.print("Set servo angle to: "); // [0 ; 170]
    Serial.println(angle);
    currentPotVal = potVal;
  }
  delay(10);
}