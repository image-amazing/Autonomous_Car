
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

#include <SoftwareSerial.h>
#include <SPI.h>
#include <WString.h>
#include <Wire.h>

#include <Adafruit_BLE.h>
#include <Adafruit_BluefruitLE_UART.h>
#include <Adafruit_LiquidCrystal.h>
#include <Adafruit_MotorShield.h>
#include <AutoFourWheelDriveCar.h>
#include <Servo.h>

#include "Config.h"

/*=========================================================================
    APPLICATION SETTINGS

    FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
   
                              Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                              running this at least once is a good idea.
   
                              When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                              Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
       
                              Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    MODE_LED_BEHAVIOUR        LED activity, valid options are
                              "DISABLE" or "MODE" or "BLEUART" or
                              "HWUART"  or "SPI"  or "MANUAL"
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE         1
    #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
    #define MODE_LED_BEHAVIOUR          "MODE"
/*=========================================================================*/

// Constants
const int                 LEFT_DIRECTION                     = 0;
const int                 RIGHT_DIRECTION                    = 1;
const unsigned int        MAX_SPEED                          = 255;
const unsigned int        MIN_SPEED                          = 0;
const unsigned int        SPEED_STEP                         = 10;
const unsigned int        FORWARD_SPEED                      = 150;
const unsigned int        RESEARCH_SPEED                     = 200;

// Constants driving the obstacles recognition
const int                 MIN_OBSTACLE_DISTANCE_FOR_STOPPING = 50;
const int                 MIN_OBSTACLE_DISTANCE_FOR_STARTING = 60;

SoftwareSerial            bluetoothControllerSS              = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);
Adafruit_BluefruitLE_UART bluetoothController(bluetoothControllerSS, BLUEFRUIT_UART_MODE_PIN, BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);

// Variables related to the car controller
Adafruit_MotorShield      AFMS                               = Adafruit_MotorShield();
Adafruit_DCMotor          *motorFrontLeft                    = AFMS.getMotor(1);
Adafruit_DCMotor          *motorFrontRight                   = AFMS.getMotor(4);
Adafruit_DCMotor          *motorRearLeft                     = AFMS.getMotor(2);
Adafruit_DCMotor          *motorRearRight                    = AFMS.getMotor(3);
AutoFourWheelDriveCar     car                                = AutoFourWheelDriveCar(motorFrontLeft, motorFrontRight, motorRearLeft, motorRearRight, FORWARD_SPEED, 2300, 10.0);
int                       carSpeed                           = FORWARD_SPEED;

// Connect via i2c, default address #0 (A0-A2 not jumpered)
Adafruit_LiquidCrystal    screen(0);

// Calibration values
const int                 MIN_DEGREES                        = 10;
const int                 MAX_DEGREES                        = 170;
int                       minFeedback;
int                       maxFeedback;

// Variables related to the servo motor
Servo                     servo;

const int                 AUTONOMOUS_MODE                    = 0;
const int                 REMOTE_CONTROLLED_MODE             = 1;
int                       controlMode                        = REMOTE_CONTROLLED_MODE;

// A small helper
void error(const __FlashStringHelper* err) {
  Serial.println(err);
  while (1);
}

// Function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// External variable declared and filled in `packetParser`.
// It will contain the bugger of data read
extern uint8_t packetbuffer[];

void setup()
{
  Serial.begin (9600);

  ////////////////////////////////////////////////////
  // Initialization of the different modules
  
  initializeBluetoothController();
  initializeLCDController();
  initializeMotorsController();
  initializeServoController();
  initializeSonarController();
}

/*
  Initialization of the bluetooth controller and connection to an app supporting  UART protocol.
  The bluetooth controller is an 'Adafruit Bluefruit LE UART Friend BLE' (https://www.adafruit.com/products/2479)
*/
void initializeBluetoothController() {
  
  Serial.print(F("Initializing the Bluetooth module: "));

  if(!bluetoothController.begin(VERBOSE_MODE)) {
    error(F("Couldn't find bluetooth controller, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println(F("OK!"));

  if(FACTORYRESET_ENABLE) {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if(!bluetoothController.factoryReset()){
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  bluetoothController.echo(false);

  Serial.println(F("Please connect your 'Becky is Back!' app to the bluetooth controller"));
  Serial.println();

  bluetoothController.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while(!bluetoothController.isConnected()) {
      delay(500);
  }

  Serial.println(F("******************************"));

  // LED Activity command is only supported from 0.6.6
  if(bluetoothController.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION))
  {
    // Change Mode LED Activity
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    bluetoothController.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // Set bluetooth to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  bluetoothController.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("******************************"));
}

void initializeLCDController() {
  
  // Set up the LCD's number of rows and columns: 
  screen.begin(16, 2);
  // Print a message to the LCD.
  screen.print("Becky est ici!");
}

void initializeMotorsController() {

  AFMS.begin();
  car.initMotors();
}

void initializeServoController() {
  
  // Initialize servo motor
  servo.attach(SERVO_CONTROL_PIN);
  calibrate(servo, SERVO_FEEDBACK_PIN, MIN_DEGREES, MAX_DEGREES);
}

void initializeSonarController() {
  
  // Initialize pin that will drive the sonar process
  pinMode(SONAR_TRIG_PIN, OUTPUT);
  pinMode(SONAR_ECHO_PIN, INPUT);
}

/*
  This function establishes the feedback values for 2 positions of the servo.
  With this information, we can interpolate feedback values for intermediate positions
*/
void calibrate(Servo servo, int analogPin, int minPos, int maxPos) {
  
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
  
  // We always have to read remote commands even when the car is in automous mode
  // to be able to switch from mode AUTONOMOUS to REMOTE CONTROLLED
  uint8_t remoteCommand = readRemotePacket();
  if(remoteCommand != 0) {
    if(isRemoteControlledFunctionalityOnly(remoteCommand) && isRemoteControlled()) {
      executeRemoteControlledCommand(remoteCommand);
    }
    else if(!isRemoteControlledFunctionalityOnly(remoteCommand)){
      changeControlMode(remoteCommand);
    }
  }
 
  if(isAutonomousMode()) {
    long distance = getDistanceToObstacle();
    if(distance <= MIN_OBSTACLE_DISTANCE_FOR_STOPPING) {
      printWhenObstacleDetected(distance);
      searchDirection();
    }
    else {
      // we can go forward
      car.forward();
    }
  }
}

uint8_t readRemotePacket() {
  
  // A remote command equal to 0 means no valid action.
  // Otherwise, the command is an unsigned int strictly superior to 0.
  uint8_t remoteCommand = 0;
  
  /* Wait for new data to arrive */
  uint8_t len = readPacket(&bluetoothController, BLE_READPACKET_TIMEOUT);
  if(len != 0 && packetbuffer[1] == 'B') {
    remoteCommand = packetbuffer[2] - '0';
  }
  
  return remoteCommand;
}

void executeRemoteControlledCommand(const uint8_t remoteCommand) {
  
  boolean buttonPressed = packetbuffer[3] - '0';
  if(buttonPressed) {
    switch(remoteCommand) {
    case COMMAND_INCREASE_SPEED:
      increaseSpeed();
      break;
    case COMMAND_DECREASE_SPEED:
      decreaseSpeed();
      break;
    case COMMAND_FORWARD:
      car.forward();
      break;
    case COMMAND_BACKWARD:
      car.backward();
      break;
    case COMMAND_TURN_LEFT:
      car.turnLeft();
      break;
    case COMMAND_TURN_RIGHT:
      car.turnRight();
      break;
    default:
      // Default case, we do nothing
      break;
    }
  }
  else {
    switch(remoteCommand) {
    case COMMAND_FORWARD:
    case COMMAND_BACKWARD:
    case COMMAND_TURN_LEFT:
    case COMMAND_TURN_RIGHT:
      car.stopMotors();
      break;
    default:
      // Default case, we do nothing
      break;
    }
  }
}

void changeControlMode(const uint8_t remoteCommand) {
  
  boolean buttonPressed = packetbuffer[3] - '0';
  if(buttonPressed) {
    if(remoteCommand == COMMAND_REMOTE_CONTROLLED_MODE) {
      switchToRemoteControlledMode();
    }
    else if(remoteCommand == COMMAND_AUTONOMOUS_MODE) {
      switchToAutonomousMode();
    }
  }
}

boolean isRemoteControlledFunctionalityOnly(const uint8_t remoteCommand) {
  if(remoteCommand == COMMAND_INCREASE_SPEED ||
     remoteCommand == COMMAND_DECREASE_SPEED ||
     remoteCommand == COMMAND_FORWARD ||
     remoteCommand == COMMAND_BACKWARD ||
     remoteCommand == COMMAND_TURN_LEFT ||
     remoteCommand == COMMAND_TURN_RIGHT) {
    // These control ony make sense in remote control
    return true;
  }
  return false;
}

boolean isRemoteControlled() {
  return controlMode == REMOTE_CONTROLLED_MODE;
}

boolean isAutonomousMode() {
  return controlMode == AUTONOMOUS_MODE;
}

void switchToRemoteControlledMode() {
  controlMode = REMOTE_CONTROLLED_MODE;
  // We reset the speed of the remote controlled mode
  car.setSpeed(carSpeed);
  car.stopMotors();
}

void switchToAutonomousMode() {
  controlMode = AUTONOMOUS_MODE;
  // We reset the default forward speed
  car.setSpeed(FORWARD_SPEED);
}

////////////////////////////////////////////////////////////////////////////////////
// Functions specific to automous mode

void searchDirection() {
  
  // First thing we do is to stop the car and to set its speed of research
  car.stopMotors();
  car.setSpeed(RESEARCH_SPEED);

  // We actually search for a new direction
  bool newDirectionFound = false;
  while(!newDirectionFound) {
    
    long distanceToObstacle = 0;
    long distancePerPositionIndex[((MAX_DEGREES - MIN_DEGREES) / 5) + 1];
    long maxDistance = 0;
    int positionIndex = 0;
    int maxDistancePositionIndex = 0;
    
    // The idea is to get all the distance for the range {MIN_DEGREES, MAX_DEGREES}
    // and then, to go to the direction where the distance was the max
    for(int pos = MIN_DEGREES; pos <= MAX_DEGREES; pos += 10) {
      
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
    
    if(maxDistance > MIN_OBSTACLE_DISTANCE_FOR_STARTING) {
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
  digitalWrite(SONAR_TRIG_PIN, HIGH);
  delayMicroseconds(20);
  // We stop the measuring process
  digitalWrite(SONAR_TRIG_PIN, LOW);
  
  // We retrieve the length of HIGH signal on the echo pin
  long duration = pulseIn(SONAR_ECHO_PIN, HIGH);
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

////////////////////////////////////////////////////////////////////////////////////
// Functions specific to remote control by bluetooth

void increaseSpeed(void) {
  // First, we determine the new speed to apply
  carSpeed += SPEED_STEP;
  if(carSpeed > MAX_SPEED) {
    carSpeed = MAX_SPEED;
  }
  // We set the speed
  car.setSpeed(carSpeed);
}

void decreaseSpeed(void) {
  // First, we determine the new speed to apply
  carSpeed -= SPEED_STEP;
  if(carSpeed < MIN_SPEED) {
    carSpeed = MIN_SPEED;
  }
  // We set the speed
  car.setSpeed(carSpeed);
}

////////////////////////////////////////////////////////////////////////////////////
// Functions common to all modes

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
