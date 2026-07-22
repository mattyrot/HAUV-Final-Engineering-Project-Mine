#include <ESP32Servo.h>

Servo camServo;
const int SERVO_PIN = 13;

void setup() {
  camServo.setPeriodHertz(50);
  camServo.attach(SERVO_PIN, 500, 2400);
  camServo.write(90);
  delay(1000);
}

void loop() {
  camServo.write(60);
  delay(1000);
  camServo.write(90);
  delay(1000);
  camServo.write(120);
  delay(1000);
  camServo.write(90);
  delay(1000);
}
