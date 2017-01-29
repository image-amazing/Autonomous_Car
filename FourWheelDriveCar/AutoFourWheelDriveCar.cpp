#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <FourWheelDriveCar.h>
#include "AutoFourWheelDriveCar.h"
    
AutoFourWheelDriveCar::AutoFourWheelDriveCar(Adafruit_DCMotor *motorFrontLeft,
                                             Adafruit_DCMotor *motorFrontRight,
                                             Adafruit_DCMotor *motorRearLeft,
                                             Adafruit_DCMotor *motorRearRight,
                                             unsigned int motorSpeed,
                                             unsigned int carSpeed,
                                             float turningRadius)
: FourWheelDriveCar(motorFrontLeft, motorFrontRight, motorRearLeft, motorRearRight, motorSpeed) {
    this->carSpeed = carSpeed;
    this->turningRadius = turningRadius;
}

void AutoFourWheelDriveCar::forwardByLength(unsigned int length) {
    this->processActionByValue(&FourWheelDriveCar::forward, &AutoFourWheelDriveCar::getIntervalFromLength, length, 1.0);
}

void AutoFourWheelDriveCar::backwardByLength(unsigned int length) {
    this->processActionByValue(&FourWheelDriveCar::backward, &AutoFourWheelDriveCar::getIntervalFromLength, length, 1.0);
}

void AutoFourWheelDriveCar::turnLeftByAngle(unsigned int angle) {
    // We need a correction factor because the right front wheel is less powerful
    this->processActionByValue(&FourWheelDriveCar::turnLeft, &AutoFourWheelDriveCar::getIntervalFromAngle, angle, 1.2);
}

void AutoFourWheelDriveCar::turnRightByAngle(unsigned int angle) {
    // We need a correction factor because the right front wheel is less powerful
    this->processActionByValue(&FourWheelDriveCar::turnRight, &AutoFourWheelDriveCar::getIntervalFromAngle, angle, 0.95);
}

void AutoFourWheelDriveCar::forwardByTime(long interval) {
    this->processActionByTime(&FourWheelDriveCar::forward, interval, 1.0);
}

void AutoFourWheelDriveCar::backwardByTime(long interval) {
    this->processActionByTime(&FourWheelDriveCar::backward, interval, 1.0);
}

void AutoFourWheelDriveCar::turnLeftByTime(long interval) {
    this->processActionByTime(&FourWheelDriveCar::turnLeft, interval, 1.2);
}

void AutoFourWheelDriveCar::turnRightByTime(long interval) {
    this->processActionByTime(&FourWheelDriveCar::turnRight, interval, 0.95);
}


void AutoFourWheelDriveCar::turn(int angle) {
    long interval = this->getIntervalFromAngle(abs(angle));
    long startTime = millis();

    if(angle < 0) {
        // We need a correction factor because the right front wheel is less powerful
        interval *= 1.2;
        this->turnLeft();
    }
    else {
        // We need a correction factor because the right front wheel is less powerful
        interval *= 0.95;
        this->turnRight();
    }
    while(millis() - startTime < interval) {}
    this->stopMotors();
}

long AutoFourWheelDriveCar::getIntervalFromLength(unsigned int length) {
    float speedCmSecond = this->carSpeed / 60.0;
    long interval = (long)(((float)length / speedCmSecond) * 1000.0);
    /*
    Serial.println("--------------");
    Serial.print("FOR LENGTH : ");
    Serial.println(length);
    Serial.print("INTERVAL : ");
    Serial.println(interval);
    Serial.println("--------------");
    */
    return interval;
}

long AutoFourWheelDriveCar::getIntervalFromAngle(unsigned int angle) {
    float length = 2.0 * 3.14 * this->turningRadius * ((float)angle/360.0);
    float speedCmSecond = this->carSpeed / 60.0;
    long interval = (long)(((float)length / speedCmSecond) * 1000.0 * 2.0);
    /*
    Serial.println("--------------");
    Serial.print("FOR ANGLE : ");
    Serial.println(angle);
    Serial.print("LENGTH : ");
    Serial.println(length);
    Serial.print("INTERVAL : ");
    Serial.println(interval);
    Serial.println("--------------");
    */
    return interval;
}

void AutoFourWheelDriveCar::processActionByValue(void(FourWheelDriveCar::*move)(),
                                                 long(AutoFourWheelDriveCar::*getInterval)(unsigned int),
                                                 unsigned int value,
                                                 float correctionFactor) {

    long interval = (this->*getInterval)(value);
    interval *= correctionFactor;
    long startTime = millis();

    (this->*move)();
    while(millis() - startTime < interval) {}
    this->stopMotors();
}

void AutoFourWheelDriveCar::processActionByTime(void(FourWheelDriveCar::*move)(),
                                                long interval,
                                                float correctionFactor) {

    interval *= correctionFactor;
    long startTime = millis();

    (this->*move)();
    while(millis() - startTime < interval) {}
    this->stopMotors();
}