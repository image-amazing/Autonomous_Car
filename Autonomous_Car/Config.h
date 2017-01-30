// SERVO MOTOR WITH FEEDBACK PIN SETTINGS
#define SERVO_CONTROL_PIN              10
#define SERVO_FEEDBACK_PIN             A0

// SONAR PIN SETTINGS
#define SONAR_TRIG_PIN                 2
#define SONAR_ECHO_PIN                 3


// BLUETOOTH CONTROLLER SETTINGS
#define BUFSIZE                        128   // Size of the read buffer for incoming data
#define VERBOSE_MODE                   true  // If set to 'true' enables debug output
#define BLE_READPACKET_TIMEOUT         500   // Timeout in ms waiting to read a response

// BLUETOOTH PIN SETTINGS
// ----------------------------------------------------------------------------------------------
// The following macros declare the pins that will be used for 'SW' serial.
// You should use this option if you are connecting the UART Friend to an UNO
// ----------------------------------------------------------------------------------------------
#define BLUEFRUIT_SWUART_RXD_PIN       9    // Required for software serial!
#define BLUEFRUIT_SWUART_TXD_PIN       13   // Required for software serial!
#define BLUEFRUIT_UART_CTS_PIN         11   // Required for software serial!
#define BLUEFRUIT_UART_RTS_PIN         8    // Optional, set to -1 if unused
#define BLUEFRUIT_UART_MODE_PIN        12   // Optional, set to -1 if unused

// REMOTE COMMANDS 
#define COMMAND_INCREASE_SPEED         1
#define COMMAND_DECREASE_SPEED         2
#define COMMAND_REMOTE_CONTROLLED_MODE 3
#define COMMAND_AUTONOMOUS_MODE        4
#define COMMAND_FORWARD                5
#define COMMAND_BACKWARD               6
#define COMMAND_TURN_LEFT              7
#define COMMAND_TURN_RIGHT             8

