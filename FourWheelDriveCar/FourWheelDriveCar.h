#ifndef _4WD_Car_h_
#define _4WD_Car_h_

#include <Wire.h>
#include <Adafruit_MotorShield.h>

class FourWheelDriveCar {

public:

    FourWheelDriveCar(Adafruit_DCMotor *motorFrontLeft,
                      Adafruit_DCMotor *motorFrontRight,
                      Adafruit_DCMotor *motorRearLeft,
                      Adafruit_DCMotor *motorRearRight,
                      unsigned int motorSpeed = 200);

    void initMotors();
    void stopMotors();
    void setSpeed(unsigned int motorSpeed);
    
    void forward();
    void backward();
    void turnLeft();
    void turnRight();
    
private:

    void initMotors(unsigned int motorSpeed);
    void initMotor(Adafruit_DCMotor *motor, unsigned int motorSpeed);
    void setMotorsSpeed();

protected:

    Adafruit_DCMotor *motorFrontLeft;
    Adafruit_DCMotor *motorFrontRight;
    Adafruit_DCMotor *motorRearLeft;
    Adafruit_DCMotor *motorRearRight;

    unsigned int motorSpeed;
};
#endif