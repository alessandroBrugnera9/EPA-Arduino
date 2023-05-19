#include <SPI.h>

// libraries to handle motor through MCP
#include <mcp_can.h>
#include <MotorHandler.h>

// setting can bus handler though SPI pins
const byte spiCSPin = 9;
MCP_CAN canHandler(spiCSPin);

// motor object instantiation
MotorHandler motor(canHandler, 0x01, 50, 0.5);

void initializeCanBus()
{
  while (CAN_OK != canHandler.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ))
  {
    Serial.println("CAN Bus initialization failed");
    Serial.println("Initializing CAN Bus again");
    delay(100);
  }
  Serial.println("CAN Bus init OK");
}

// ------------------------------------------------------------------------------------
// Related to receiveing commands from Serial
// Define command constants
const char POSITION_CMD = 'p';
const char TORQUE_MODE_CMD = 't';
const char ZERO_CMD = 'z';
const char TEST_CMD = 'a';
const char EXIT_MOTOR_CMD = 'e';
const char SET_MOTOR_CMD = 'm';

// Define delimiter constant
const char DELIMITER = '/';

String getStringInSerialBuffer()
{
  delay(10);
  String bufferString = "";
  // listen to Serial port until it is empty
  while (Serial.available())
  {
    bufferString += (char)Serial.read();
  }

  return bufferString;
}

void handleCommand(String inputString)
{
  char command = inputString.charAt(0);
  switch (command)
  {
  case POSITION_CMD:
  {
    int delimiter = inputString.indexOf(DELIMITER);

    // check if command is correct
    if (delimiter > 0)
    {
      float pos = inputString.substring(delimiter + 1).toFloat(); // Get position parameter
      Serial.print("Position in rad: ");
      Serial.println(pos);
      motor.moveMotor(pos);
    }
    else
    {
      Serial.println("Wrong position command. Try p/*ANGLE*.");
    }
  }
  break;
  case TORQUE_MODE_CMD:
  {
    motor.setTorqueMode(2);
  }
  break;
  case ZERO_CMD:
  {
    motor.resetPosition();
  }
  break;
  case TEST_CMD:
  {
    Serial.println("Test command received");
  }
  break;
  case EXIT_MOTOR_CMD:
  {
    motor.exitMotorMode();
    Serial.println("Test command received");
  }
  break;
  case SET_MOTOR_CMD:
  {
    int delimiter1 = inputString.indexOf(DELIMITER);
    int delimiter2 = inputString.indexOf(DELIMITER, delimiter1 + 1);
    int delimiter3 = inputString.indexOf(DELIMITER, delimiter2 + 1);

    // check if command is correct
    if (delimiter1 > 0 && delimiter2 > 0 && delimiter3 > 0)
    {
      float position = inputString.substring(delimiter1 + 1, delimiter2).toFloat();
      float velocity = inputString.substring(delimiter2 + 1, delimiter3).toFloat();
      float torque = inputString.substring(delimiter3 + 1).toFloat();

      Serial.print("Position in rad: ");
      Serial.println(position);
      Serial.print("velocity in rad/s: ");
      Serial.println(velocity);
      Serial.print("Torque in Nm: ");
      Serial.println(torque);

      // Call motor handler method to set motor parameters
      motor.setPositionFull(position, velocity, torque);
    }
    else
    {
      Serial.println("Wrong motor command. Try m/*ANGLE*/*VELOCITY*/*TORQUE*.");
    }
  }
  break;
  default:
  {
    Serial.println("Invalid command!"); // Print error message
  }
  break;
  }
}
// ------------------------------------------------------------------------------------

void setup()
{
  Serial.begin(9600); // Begin Serial port to talk to computer and receive commands

  // Begin the CAN Bus and set frequency to 8 MHz and baudrate of 1000kb/s  and the masks and filters disabled.
  initializeCanBus();
  canHandler.setMode(MCP_NORMAL); // Change to normal mode to allow messages to be transmitted and received

  // Preparing motor to listen to commands
  motor.exitMotormode(canHandler);
  motor.setMotormode(canHandler);
  motor.setZero(canHandler);
}

void loop()
{
  // check if there is a command in the Serial buffer
  if (Serial.available())
  {
    String inputString = getStringInSerialBuffer();
    handleCommand(inputString);
  }
  delay(100);
}
