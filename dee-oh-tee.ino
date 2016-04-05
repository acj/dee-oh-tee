#include <Servo.h>

const int buttonPin = 10;
const int laserPin = 13;

const int MIN_SERVO_DEGREES = 0;
const int MAX_SERVO_DEGREES = 180;
const int MIN_NORM_DEGREES = -90;
const int MAX_NORM_DEGREES = 90;
const int USABLE_DEGREES = MAX_SERVO_DEGREES - MIN_SERVO_DEGREES;

const int CLOCKWISE = 0;
const int COUNTERCLOCKWISE = 1;

const int SERVO_DELAY = 15; // millis

Servo panServo;
Servo tiltServo;

void setup() {
  Serial.begin(9600);
  
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(laserPin, OUTPUT);
  
  // 0, 2, 3, 4, 5, 6, 7, 10, 12, 13
  panServo.attach(2);
  tiltServo.attach(4);
}

int convertNormalizedAngleToServoAngle(int normalizedAngle) {
  return 90 - normalizedAngle;
}

int convertServoAngleToNormalizedAngle(int servoAngle) {
  return 90 - servoAngle;
}

void moveToCenter(Servo servo) {
  servo.write(MAX_SERVO_DEGREES / 2);
}

void moveToPoint(int targetX, int targetY, int numberOfSteps) {
  const int startX = convertServoAngleToNormalizedAngle(panServo.read());
  const int startY = convertServoAngleToNormalizedAngle(tiltServo.read());

  const int xDisplacement = abs(targetX - startX);
  const int yDisplacement = abs(targetY - startY);

  const int xDirectionSign = targetX >= startX ? 1 : -1;
  const int yDirectionSign = targetY >= startY ? 1 : -1;

  float slope = (float)yDisplacement / (float)xDisplacement;

  float x = startX;
  float y = startY;
  
  for (int i = 0; i < numberOfSteps; i++) {
    x += xDirectionSign * ((float)xDisplacement / (float)numberOfSteps);
    y += yDirectionSign * ((float)yDisplacement / (float)numberOfSteps);

    panServo.write(convertNormalizedAngleToServoAngle(x));
    tiltServo.write(convertNormalizedAngleToServoAngle(y));
    delay(SERVO_DELAY);
  }
}

void drawCircle(int centerX, int centerY, int radius, int xDegreesPerCycle, int direction) {
  drawEllipse(centerX, centerY, radius, radius, xDegreesPerCycle, direction);
}

void drawEllipse(int centerX, int centerY, int radiusX, int radiusY, int xDegreesPerCycle, int direction) {
  const int directionSign = (direction == 0) ? 1 : -1;
  const int maxDisplacementFromCenter = USABLE_DEGREES / 2;

  const int startX = centerX - radiusX;
  const int endX = centerX + radiusX;

  for (int targetDegrees = startX; targetDegrees <= endX; targetDegrees += xDegreesPerCycle) {
    const int targetPanServoDegrees = convertNormalizedAngleToServoAngle(targetDegrees);
    
    panServo.write(targetPanServoDegrees);

    const int targetTiltDegrees = centerY + directionSign * sqrt(pow(radiusY, 2) * (1.0 - pow(targetDegrees - centerX, 2) / pow(radiusX, 2)));
    const int targetTiltServoDegrees = convertNormalizedAngleToServoAngle(targetTiltDegrees);
    tiltServo.write(targetTiltServoDegrees);

    delay(SERVO_DELAY);
  }

  for (int targetDegrees = endX; targetDegrees > startX; targetDegrees -= xDegreesPerCycle) {
    const int targetPanServoDegrees = convertNormalizedAngleToServoAngle(targetDegrees);
    
    panServo.write(targetPanServoDegrees);

    const int targetTiltDegrees = centerY - directionSign * sqrt(pow(radiusY, 2) * (1.0 - pow(targetDegrees - centerX, 2) / pow(radiusX, 2)));
    const int targetTiltServoDegrees = convertNormalizedAngleToServoAngle(targetTiltDegrees);
    tiltServo.write(targetTiltServoDegrees);

    delay(SERVO_DELAY);
  }
}

void drawRectangle(int startX, int startY, int width, int height, int xDegreesPerCycle) {
  if (startX + width > USABLE_DEGREES || startY + height > USABLE_DEGREES) {
    Serial.print("The requested rectangle goes beyond the servo's usable range");
    Serial.println();
    return;
  }

  moveToPoint(startX, startY, xDegreesPerCycle);
  moveToPoint(startX + width, startY, xDegreesPerCycle);
  moveToPoint(startX + width, startY + height, xDegreesPerCycle);
  moveToPoint(startX, startY + height, xDegreesPerCycle);
  moveToPoint(startX, startY, xDegreesPerCycle);
}

void performRangeTest() {
  moveToPoint(0, 0, 50);
  moveToPoint(-90, 90, 50);
  moveToPoint(-90, -90, 50);
  moveToPoint(90, -90, 50);
  moveToPoint(90, 90, 50);
  moveToPoint(-90, 90, 50);
  moveToPoint(0, 0, 50);
}

// Laser control
void enableLaser() {
  digitalWrite(laserPin, HIGH);
}

void disableLaser() {
  digitalWrite(laserPin, LOW);
}

void loop() {
  const int buttonState = digitalRead(buttonPin);

  if (buttonState == LOW) {
    enableLaser();

//    performRangeTest();

//    drawCircle(0, 0, numberOfUsableDegrees / 2, 15, CLOCKWISE);

    drawEllipse(0, 0, USABLE_DEGREES / 4, USABLE_DEGREES / 4, 3, CLOCKWISE);

    disableLaser();
  }
}
