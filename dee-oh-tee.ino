#include <Servo.h>

const int buttonPin = 10;

const int minPanAngle = 700;
const int maxPanAngle = 2300;

Servo panServo;
Servo tiltServo;

int panAngleDegrees;
int tiltServoDegrees;

void setup() {
  pinMode(buttonPin, INPUT_PULLUP);
  
  // 0, 2, 3, 4, 5, 6, 7, 10, 12, 13
  panServo.attach(2);
  tiltServo.attach(4);
}

void loop() {
  int buttonState = digitalRead(buttonPin);

  if (buttonState == LOW) {
    for (panAngleDegrees = minPanAngle; panAngleDegrees <= maxPanAngle; panAngleDegrees += 3) {
      panServo.writeMicroseconds(panAngleDegrees);
      tiltServo.writeMicroseconds(panAngleDegrees);
      delay(15);
    }
    for (panAngleDegrees = maxPanAngle; panAngleDegrees >= minPanAngle; panAngleDegrees -= 3) {
      panServo.writeMicroseconds(panAngleDegrees);
      tiltServo.writeMicroseconds(panAngleDegrees);
      delay(15);
    }
  }
}
