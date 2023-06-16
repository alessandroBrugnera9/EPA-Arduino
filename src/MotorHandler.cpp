#include "MotorHandler.h"

MotorHandler::MotorHandler(MCP_CAN &CAN, int canId, float _kp, float _kd) : gim8115(canId, _kp, _kd), canHandler(CAN)
{
  baseVelocity = 0;
  baseTorque = 0;
}

void MotorHandler::moveMotor(float newPos)
{
  // sending new position to motor library using predefined velocity and torque
  normalSet(canHandler, newPos, baseVelocity, baseTorque);
}

void MotorHandler::exitMotorMode()
{
  exitMotormode(canHandler);
}
void MotorHandler::enterMotorMode()
{
  setMotormode(canHandler);
}

void MotorHandler::setTorqueMode(float tarTor)
{
  // Call the private functions to build the individual packages
  float *torPackage = buildTorquePackage(tarTor);

  //----------------------------------------------------------------------------//
  // Sending data//
  unsigned char buf[8];
  memset(buf, 0, sizeof(buf)); //setting array to 0
  buf[6] = torPackage[0];
  buf[7] = torPackage[1];

  byte sndStat = canHandler.sendMsgBuf(getId(), 0, 8, buf);
}

void MotorHandler::resetPosition()
{
  setZero(canHandler);
}

void MotorHandler::setPositionFull(float newPos, float velocity, float torque)
{
  normalSet(canHandler, newPos, velocity, torque);
}

void MotorHandler::setBaseVelocity(float vel)
{
  baseVelocity = vel;
}

void MotorHandler::setBaseTorque(float trq)
{
  baseTorque = trq;
}

void MotorHandler::printPrettyResponse(motorResponse res)
{
  char msgString2[64];     // Array to store serial string
  char posString[10];      // Position string
  char velocityString[10]; // Velocity string
  char currentString[10];  // Current string

  // Transform to string to be printable with snprintf
  snprintf(posString, sizeof(posString), "%.3f", res.position);
  snprintf(velocityString, sizeof(velocityString), "%.3f", res.velocity);
  snprintf(currentString, sizeof(currentString), "%.3f", res.current);

  snprintf(msgString2, sizeof(msgString2), "Position: %s  Velocity: %s  Current: %s", posString, velocityString, currentString);
  Serial.println(msgString2);
}