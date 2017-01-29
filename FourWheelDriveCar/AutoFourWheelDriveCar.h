#ifndef _AUTO_4WD_Car_h_
#define _AUTO_4WD_Car_h_

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <FourWheelDriveCar.h>

class AutoFourWheelDriveCar : public FourWheelDriveCar {

public:

    // Car speed in cm/min
    // Turning radius in cm
    AutoFourWheelDriveCar(Adafruit_DCMotor *motorFrontLeft,
                          Adafruit_DCMotor *motorFrontRight,
                          Adafruit_DCMotor *motorRearLeft,
                          Adafruit_DCMotor *motorRearRight,
                          unsigned int motorSpeed,
                          unsigned int carSpeed,
                          float turningRadius);

    // Length in cm
    void forwardByLength(unsigned int length);
    // Length in cm
    void backwardByLength(unsigned int length);
    // Angle in degree Celsius 
    void turnLeftByAngle(unsigned int angle);
    // Angle in degree Celsius
    void turnRightByAngle(unsigned int angle);

    // Interval in milliseconds
    void forwardByTime(long interval);
    // Interval in milliseconds
    void backwardByTime(long interval);
    // Interval in milliseconds
    void turnLeftByTime(long interval);
    // Interval in milliseconds
    void turnRightByTime(long interval);

    // Angle in degree Celsius
    void turn(int angle);

private:

    // Length in cm
    // Interval in milliseconds
    long getIntervalFromLength(unsigned int length);
    // Length in cm
    // Interval in milliseconds
    long getIntervalFromAngle(unsigned int angle);
    
    void processActionByValue(void(FourWheelDriveCar::*move)(),
                              long(AutoFourWheelDriveCar::*getInterval)(unsigned int),
                              unsigned int value,
                              float correctionFactor);

    void processActionByTime(void(FourWheelDriveCar::*move)(),
                              long interval,
                              float correctionFactor);

private:

    unsigned int carSpeed;
    float turningRadius;
};
#endif