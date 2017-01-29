#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "FourWheelDriveCar.h"

FourWheelDriveCar::FourWheelDriveCar(Adafruit_DCMotor *motorFrontLeft,
                                     Adafruit_DCMotor *motorFrontRight,
                                     Adafruit_DCMotor *motorRearLeft,
                                     Adafruit_DCMotor *motorRearRight,
                                     unsigned int motorSpeed) {
    this->motorFrontLeft = motorFrontLeft;
    this->motorFrontRight = motorFrontRight;
    this->motorRearLeft = motorRearLeft;
    this->motorRearRight = motorRearRight;
    this->motorSpeed = motorSpeed;
}

void FourWheelDriveCar::initMotors(unsigned int motorSpeed) {
    this->initMotor(this->motorFrontLeft, motorSpeed);
    this->initMotor(this->motorFrontRight, motorSpeed);
    this->initMotor(this->motorRearLeft, motorSpeed);
    this->initMotor(this->motorRearRight, motorSpeed);
}

void FourWheelDriveCar::initMotors() {
    this->initMotors(this->motorSpeed);
}

void FourWheelDriveCar::initMotor(Adafruit_DCMotor *motor, unsigned int motorSpeed) {
    motor->setSpeed(motorSpeed);
    motor->run(RELEASE);
}

void FourWheelDriveCar::stopMotors() {
    this->motorFrontLeft->run(RELEASE);
    this->motorFrontRight->run(RELEASE);
    this->motorRearLeft->run(RELEASE);
    this->motorRearRight->run(RELEASE);
    delay(100);
}

void FourWheelDriveCar::setSpeed(unsigned int motorSpeed) {
    // We update the motor speed internal member
    this->motorSpeed = motorSpeed;
    // We update the speed of all car motors
    this->setMotorsSpeed();
}

void FourWheelDriveCar::setMotorsSpeed() {
    this->motorFrontLeft->setSpeed(this->motorSpeed);
    this->motorFrontRight->setSpeed(this->motorSpeed);
    this->motorRearLeft->setSpeed(this->motorSpeed);
    this->motorRearRight->setSpeed(this->motorSpeed);
}

void FourWheelDriveCar::forward() {
    this->motorFrontLeft->run(FORWARD);
    this->motorFrontRight->run(FORWARD);
    this->motorRearLeft->run(FORWARD);
    this->motorRearRight->run(FORWARD);
}

void FourWheelDriveCar::backward() {
    this->motorFrontLeft->run(BACKWARD);
    this->motorFrontRight->run(BACKWARD);
    this->motorRearLeft->run(BACKWARD);
    this->motorRearRight->run(BACKWARD);
}

void FourWheelDriveCar::turnLeft() {
    this->motorFrontRight->run(FORWARD);
    this->motorRearRight->run(FORWARD);
    this->motorFrontLeft->run(BACKWARD);
    this->motorRearLeft->run(BACKWARD);
}

void FourWheelDriveCar::turnRight() {
    this->motorFrontLeft->run(FORWARD);
    this->motorRearLeft->run(FORWARD);
    this->motorFrontRight->run(BACKWARD);
    this->motorRearRight->run(BACKWARD);
}
