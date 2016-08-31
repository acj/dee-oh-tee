/**
 * A simple sketch for drawing fun shapes with a laser that's attached to a pan/tilt
 * servo. (Cat not included.)
 * 
 * This sketch is designed for the Trinket microcontroller, which is Arduino-compatible
 * and made by Adafruit. It should be easily portable to any Arduino platform.
 * 
 * This type of servo works on angles, but I find it easier to think in Cartesian 
 * coordinates. Most of the functions in this library take x- and y-coordinates. The
 * origin (center) is at x = 0, y = 0. There are USABLE_DEGREES / 2 units of space
 * between the origin and the limits of each servo's range in either direction.
 * 
 * Have fun!
 * 
 * -- Adam Jensen <acjensen at gmail>
 */

#include "Adafruit_SoftServo.h"

const int PAN_SERVO_PIN = 3;
const int TILT_SERVO_PIN = 4;
const int LASER_PIN = 0;

const int MIN_SERVO_DEGREES = 0;
const int MAX_SERVO_DEGREES = 180;
const int MIN_NORM_DEGREES = -90;
const int MAX_NORM_DEGREES = 90;
const int USABLE_DEGREES = MAX_SERVO_DEGREES - MIN_SERVO_DEGREES;

const int CLOCKWISE = 0;
const int COUNTERCLOCKWISE = 1;

const int SERVO_DELAY = 25; // millis

typedef Adafruit_SoftServo Servo;
Servo panServo;
Servo tiltServo;

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
    const int targetTiltDegrees = centerY + directionSign * sqrt(pow(radiusY, 2) * (1.0 - pow(targetDegrees - centerX, 2) / pow(radiusX, 2)));

    moveToPoint(targetDegrees, targetTiltDegrees, xDegreesPerCycle);
  }

  for (int targetDegrees = endX; targetDegrees > startX; targetDegrees -= xDegreesPerCycle) {
    const int targetPanServoDegrees = convertNormalizedAngleToServoAngle(targetDegrees);
    const int targetTiltDegrees = centerY - directionSign * sqrt(pow(radiusY, 2) * (1.0 - pow(targetDegrees - centerX, 2) / pow(radiusX, 2)));

    moveToPoint(targetDegrees, targetTiltDegrees, xDegreesPerCycle);
  }
}

void drawZigZag(int startX, int endX, int yDisplacementFromCenter, int numberOfZigs, int xDegreesPerCycle) {
  float xDistanceBetweenZigs = (float)(endX - startX) / (float)numberOfZigs;
  for (int i = 0; i < numberOfZigs; i++) {
    const int zigSign = (i % 2 == 0) ? 1 : -1;
    moveToPoint(startX + i * xDistanceBetweenZigs, zigSign * yDisplacementFromCenter, xDegreesPerCycle);
  }
}

void drawSineWave(int startX, int endX, int yOffset, int amplitude, int numberOfPeriods) {
  const float lengthOfPeriod = (float)(endX - startX) / numberOfPeriods;
  const float lengthOfPeriodInRadians = lengthOfPeriod / (2.0 * M_PI);

  for (int x = startX; x < endX; x++) {
    moveToPoint(x, amplitude * sinf((float)x / lengthOfPeriodInRadians) + yOffset, 1);
  }

  for (int x = endX; x > startX; x--) {
    moveToPoint(x, -amplitude * sinf((float)x / lengthOfPeriodInRadians) + yOffset, 1);
  }
}

void drawRectangle(int startX, int startY, int width, int height, int xDegreesPerCycle) {
  if (startX + width > USABLE_DEGREES || startY + height > USABLE_DEGREES) {
    return;
  }

  moveToPoint(startX, startY, xDegreesPerCycle);
  moveToPoint(startX + width, startY, xDegreesPerCycle);
  moveToPoint(startX + width, startY + height, xDegreesPerCycle);
  moveToPoint(startX, startY + height, xDegreesPerCycle);
  moveToPoint(startX, startY, xDegreesPerCycle);
}

void performRangeTest() {
  moveToPoint(0, 0, 15);
  drawRectangle(-USABLE_DEGREES / 2, -USABLE_DEGREES / 2, USABLE_DEGREES, USABLE_DEGREES, 15);
  moveToPoint(0, 0, 15);
}

// Laser control
void enableLaser() {
  digitalWrite(LASER_PIN, HIGH);
}

void disableLaser() {
  digitalWrite(LASER_PIN, LOW);
}

void setup() {
  pinMode(LASER_PIN, OUTPUT);
  
  panServo.attach(PAN_SERVO_PIN);
  tiltServo.attach(TILT_SERVO_PIN);
}

void loop() {
  // Set up timer interrupt
  OCR0A = 0xAF;            // any number is OK
  TIMSK |= _BV(OCIE0A);    // Turn on the compare interrupt (below!)

  enableLaser();

  performRangeTest();

//  drawSineWave(-USABLE_DEGREES / 2, USABLE_DEGREES / 2, USABLE_DEGREES / 4, USABLE_DEGREES / 8, 2);
//
//  drawCircle(0, 0, USABLE_DEGREES / 2, 15, CLOCKWISE);
//
//  drawEllipse(0, 0, USABLE_DEGREES / 4, USABLE_DEGREES / 4, 3, CLOCKWISE);
//
//  drawZigZag(-USABLE_DEGREES / 2, USABLE_DEGREES / 2, USABLE_DEGREES / 4, 20, 15);

  disableLaser();
}

// We'll take advantage of the built in millis() timer that goes off
// to keep track of time, and refresh the servo every 20 milliseconds
volatile uint8_t counter = 0;
SIGNAL(TIMER0_COMPA_vect) {
  // this gets called every 2 milliseconds
  counter += 2;
  // every 20 milliseconds, refresh the servos!
  if (counter >= 20) {
    counter = 0;
    panServo.refresh();
    tiltServo.refresh();
  }
}
