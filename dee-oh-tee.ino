#include <Servo.h>

const int buttonPin = 10;

const int minServoAngle = 700;
const int maxServoAngle = 2300;
const int numberOfUsableDegrees = maxServoAngle - minServoAngle;
const int centerServoAngle = minServoAngle + numberOfUsableDegrees / 2;

Servo panServo;
Servo tiltServo;

void setup() {
  Serial.begin(9600);
  
  pinMode(buttonPin, INPUT_PULLUP);
  
  // 0, 2, 3, 4, 5, 6, 7, 10, 12, 13
  panServo.attach(2);
  tiltServo.attach(4);
}

int convertNormalizedAngleToServoAngle(int normalizedAngle) {
  return normalizedAngle + minServoAngle;
}

void drawCircle(int centerX, int centerY, int radius, int xDegreesPerCycle) {
  drawEllipse(centerX, centerY, radius, radius, xDegreesPerCycle);
}

void drawEllipse(int centerX, int centerY, int radiusX, int radiusY, int xDegreesPerCycle) {
  const int maxDisplacementFromCenter = numberOfUsableDegrees / 2;
  const int tiltAngleCenter = minServoAngle + maxDisplacementFromCenter;

  for (int targetDegrees = 0; targetDegrees <= numberOfUsableDegrees; targetDegrees += xDegreesPerCycle) {
    const int targetPanServoDegrees = convertNormalizedAngleToServoAngle(targetDegrees);
    
    panServo.writeMicroseconds(targetPanServoDegrees);

    const int targetTiltDegrees = (maxDisplacementFromCenter + centerY) - sqrt(pow(radiusY, 2) * (1.0 - pow(targetDegrees - maxDisplacementFromCenter - centerX, 2) / pow(radiusX, 2)));
    const int targetTiltServoDegrees = convertNormalizedAngleToServoAngle(targetTiltDegrees);
    tiltServo.writeMicroseconds(targetTiltServoDegrees);

    delay(15);
  }

  for (int targetDegrees = numberOfUsableDegrees; targetDegrees > 0; targetDegrees -= xDegreesPerCycle) {
    const int targetPanServoDegrees = convertNormalizedAngleToServoAngle(targetDegrees);
    
    panServo.writeMicroseconds(targetPanServoDegrees);

    const int targetTiltDegrees = (maxDisplacementFromCenter + centerY ) + sqrt(pow(radiusY, 2) * (1.0 - pow(targetDegrees - maxDisplacementFromCenter - centerX, 2) / pow(radiusX, 2)));
    const int targetTiltServoDegrees = convertNormalizedAngleToServoAngle(targetTiltDegrees);
    tiltServo.writeMicroseconds(targetTiltServoDegrees);

    delay(15);
  }
}

void performRangeTest() {
  const int maxDisplacementFromCenter = numberOfUsableDegrees / 2;
  
  for (int targetDegrees = -maxDisplacementFromCenter; targetDegrees <= maxDisplacementFromCenter; targetDegrees += 3) {
    const int targetServoDegrees = convertNormalizedAngleToServoAngle(targetDegrees);
    panServo.writeMicroseconds(targetServoDegrees);
    tiltServo.writeMicroseconds(targetServoDegrees);
    delay(15);
  }
  for (int targetDegrees = maxDisplacementFromCenter; targetDegrees >= -maxDisplacementFromCenter; targetDegrees -= 3) {
    const int targetServoDegrees = convertNormalizedAngleToServoAngle(targetDegrees);
    panServo.writeMicroseconds(targetServoDegrees);
    tiltServo.writeMicroseconds(targetServoDegrees);
    delay(15);
  }
}

void loop() {
  const int buttonState = digitalRead(buttonPin);

  if (buttonState == LOW) {
    const int initialTiltAngle = tiltServo.read();
    const int initialPanAngle = panServo.read();
  
//    performRangeTest();

//    drawCircle(0, 0, numberOfUsableDegrees / 2, 15);

    drawEllipse(0, -(numberOfUsableDegrees / 3), numberOfUsableDegrees / 2, numberOfUsableDegrees / 8, 15);
  }
}
