
#ifdef DEBUG
 #define DEBUG_PRINT(x)       Serial.print (x)
 #define DEBUG_PRINTDEC(x)    Serial.print (x, DEC)
 #define DEBUG_PRINTDECLN(x)  Serial.println (x, DEC)
 #define DEBUG_PRINTLN(x)     Serial.println (x)
#else
 #define DEBUG_PRINT(x)
 #define DEBUG_PRINTDEC(x)
 #define DEBUG_PRINTDECLN(x)
 #define DEBUG_PRINTLN(x) 
#endif

#include <WString.h>
#include <Wire.h>

#include <Adafruit_LiquidCrystal.h>
#include <Adafruit_MotorShield.h>
#include <AutoFourWheelDriveCar.h>
#include <Servo.h> 

// Constants
const int LEFT_DIRECTION = 0;
const int RIGHT_DIRECTION = 1;
const unsigned int FORWARD_SPEED = 150;
const unsigned int RESEARCH_SPEED = 200;

// Pins driving the servo motor
const int servoPin = 10;
const int feedbackPin = A0;

// Pins driving sonar controller
const int trigPin = 2;
const int echoPin = 3;

// Constants driving the obstacles recognition
const int minObstacleDistanceForStopping = 50;
const int minObstacleDistanceForStarting = 60;

// Variables related to the car controller
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motorFrontLeft = AFMS.getMotor(1);
Adafruit_DCMotor *motorFrontRight = AFMS.getMotor(4);
Adafruit_DCMotor *motorRearLeft = AFMS.getMotor(2);
Adafruit_DCMotor *motorRearRight = AFMS.getMotor(3);
AutoFourWheelDriveCar car = AutoFourWheelDriveCar(motorFrontLeft, motorFrontRight, motorRearLeft, motorRearRight, FORWARD_SPEED, 2300, 10.0);

// Connect via i2c, default address #0 (A0-A2 not jumpered)
Adafruit_LiquidCrystal screen(0);

// Calibration values
const int minDegrees = 10;
const int maxDegrees = 170;
int minFeedback;
int maxFeedback;

// Variables related to the servo motor
Servo servo;


void setup()
{
  Serial.begin (9600);

  // Set up the LCD's number of rows and columns: 
  screen.begin(16, 2);
  // Print a message to the LCD.
  screen.print("Becky tes ici!");
  
  // Initialize motors stuff
  AFMS.begin();
  car.initMotors();
  
  // Initialize servo motor
  servo.attach(servoPin);
  calibrate(servo, feedbackPin, minDegrees, maxDegrees);
  
  // Initialize pin that will drive the sonar process
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

/*
  This function establishes the feedback values for 2 positions of the servo.
  With this information, we can interpolate feedback values for intermediate positions
*/
void calibrate(Servo servo, int analogPin, int minPos, int maxPos)
{
  // Move to the minimum position and record the feedback value
  servo.write(minPos);
  delay(2000); // make sure it has time to get there and settle
  minFeedback += analogRead(analogPin);
  
  // Move to the maximum position and record the feedback value
  servo.write(maxPos);
  delay(2000); // make sure it has time to get there and settle
  maxFeedback += analogRead(analogPin);
  
  // We set the servo motor to its default position
  servo.write(90);
  delay(100);
}

void loop() {
  long distance = getDistanceToObstacle();
  if(distance <= minObstacleDistanceForStopping) {
    printWhenObstacleDetected(distance);
    searchDirection();
  }
  else {
    // we can go forward 
    car.forward();
  }  
}

void searchDirection() {
  
  // First thing we do is to stop the car and to set its speed of research
  car.stopMotors();
  car.setSpeed(RESEARCH_SPEED);

  // We actually search for a new direction
  bool newDirectionFound = false;
  while(!newDirectionFound) {
    
    long distanceToObstacle = 0;
    long distancePerPositionIndex[((maxDegrees - minDegrees) / 5) + 1];
    long maxDistance = 0;
    int positionIndex = 0;
    int maxDistancePositionIndex = 0;
    
    // The idea is to get all the distance for the range {minDegrees, maxDegrees}
    // and then, to go to the direction where the distance was the max
    for(int pos = minDegrees; pos <= maxDegrees; pos += 10) {
      
      // Move the servo to the new position
      servo.write(pos);
      delay(200);
      
      // Measure the distance to the closest obstacle
      distanceToObstacle = getDistanceToObstacle();
      distancePerPositionIndex[positionIndex] = distanceToObstacle;
      
      if(distanceToObstacle > maxDistance && distanceToObstacle <= 400) {
        maxDistance = distanceToObstacle;
        maxDistancePositionIndex = positionIndex;
      }
      positionIndex++;
      
      printWhenEvaluatingNewPosition(pos, distanceToObstacle);
    }
     
    // We reset the servo motor to its default position
    servo.write(90);
    delay(100);
    
    if(maxDistance > minObstacleDistanceForStarting) {
      newDirectionFound = true;      
      
      // We have to turn the car to that position and go forward
      int targetAngle = maxDistancePositionIndex * 10 + 10;
      printWhenNewPositionFound(targetAngle, maxDistance);
      
      turnToNewPosition(targetAngle, maxDistance);
    }
    else {
      turnRandomly();
    }
  }

  printWhenGoingForward();
  // Finally, we reset the forwar speed and go forward
  car.setSpeed(FORWARD_SPEED);
  car.forward(); 
}

long getDistanceToObstacle() {
  // A measuring process is triggered by an HIGH during 10 ms on the trig pin
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(20);
  // We stop the measuring process
  digitalWrite(trigPin, LOW);
  
  // We retrieve the length of HIGH signal on the echo pin
  long duration = pulseIn(echoPin, HIGH);
  // distance = 17/1000 cm * duration - See http://www.gotronic.fr/pj2-hc-sr04-utilisation-avec-picaxe-1343.pdf
  long distance = (long) (17.0 / 1000.0 * (float) duration);
  return distance;
}

void turnToNewPosition(int targetAngle, long targetDistance) {
  // First of all, we have to determine if we want to go to the left or to the right
  bool turnLeft = true;
  if(targetAngle > 90) {
    turnLeft = false;
    // Working on the absolute value
    targetAngle -= 90;
  }
  
  // We rotate from the initial angle
  if(turnLeft) {
    int angle = abs(90 - targetAngle);
    printWhenTurningLeft(angle);
    car.turnLeftByAngle(angle);
  }
  else {
    printWhenTurningRight(targetAngle);
    car.turnRightByAngle(targetAngle);
  }
}

void turnRandomly() {
  // We have to turn the car to 90 degree and try to find a new position
  int direction = random(2);
  if(direction == LEFT_DIRECTION) {
    car.turnLeftByAngle(90);
  }
  else {
    car.turnRightByAngle(90);
  }
}

void printToFirstLineOfScreen(const String& text) {
  screen.setCursor(0, 0);
  screen.print(text);
}

void printToSecondLineOfScreen(const String& text) {
  screen.setCursor(0, 1);
  screen.print(text);
}

void printToScreen(const String& textFirstLine, const String& textSecondLine) {
  printToFirstLineOfScreen(textFirstLine);
  printToSecondLineOfScreen(textSecondLine);
}

void printWhenObstacleDetected(long distance) {
#ifdef DEBUG
  screen.clear();
  printToScreen("Obstacle", String("détecté - " + String(distance, DEC)));
#else
  printToScreenWhenObstacleDetected();
#endif
}
    
void printToScreenWhenObstacleDetected() {
  screen.clear();
  
  int reply = random(8);
  switch(reply) {
    case 0:
      printToScreen("Allo, qui", "est la?");
      break;
    case 1:
      printToScreen("C'est Milan,", "heinG??");
      break;
    case 2:
      printToScreen("Nooooooooon", "Novak non");
      break;
    case 3:
      printToScreen("A qui sont", "ces pieds?");
      break;
    case 4:
      printToScreen("Lucas! je te", "vois!!");
      break;
    case 5:
      printToScreen("Hum... ca", "sent pas bon!");
      break;
    case 6:
      printToScreen("Chercher...", "toujours...");
      break;
    case 7:
      printToScreen("Que dirait", "Elon Musk?");
      break;
  }
}

void printWhenEvaluatingNewPosition(int position, long distanceToObstacle) {
  // Some debugging macro that can be activated at compiling time
  DEBUG_PRINT("Angle: ");
  DEBUG_PRINTDEC(pos);
  DEBUG_PRINT(", Dist: ");
  DEBUG_PRINTDECLN(distanceToObstacle);
}

void printWhenNewPositionFound(int targetAngle, long targetDistance) {
  // Some debugging macro that can be activated at compiling time
  DEBUG_PRINTLN("!! NEW POSITION FOUND !!");
  DEBUG_PRINT("Angle: ");
  DEBUG_PRINTDEC(targetAngle);
  DEBUG_PRINT(", Dist: ");
  DEBUG_PRINTDECLN(targetDistance);
  
  // Debug information sent to the lcd screen too
  #ifdef DEBUG
    screen.clear();
    printToFirstLineOfScreen(String(String("A-D: " + String(targetAngle, DEC)) + String("-" + String(targetDistance, DEC))));
  #endif
}

void printWhenGoingForward() {
  // Funny sentence when moving
  screen.clear();
  printToScreen("Yeah", "Tout Schuss!!");
}

void printWhenTurningLeft(int angle) {
  // Some debugging macro that can be activated at compiling time
  DEBUG_PRINTLN("!! TURN LEFT !!");
  DEBUG_PRINT("Actual angle: ");
  DEBUG_PRINTDECLN(angle);
  // Debug information sent to the lcd screen too
  #ifdef DEBUG
    printToSecondLineOfScreen(String("Left, angle: " + String(angle, DEC)));
  #endif
}

void printWhenTurningRight(int angle) {
  // Some debugging macro that can be activated at compiling time
  DEBUG_PRINTLN("!! TURN RIGHT !!");
  DEBUG_PRINT("Actual angle: ");
  DEBUG_PRINTDECLN(angle);
  // Debug information sent to the lcd screen too
  #ifdef DEBUG
    printToSecondLineOfScreen(String("Right, angle: " + String(angle, DEC)));
  #endif
}
