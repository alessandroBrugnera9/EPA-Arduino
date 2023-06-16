#include "MotorHandler.h"

MotorHandler::MotorHandler(MCP_CAN &CAN, int canId, float _kp, float _kd) : gim8115(canId, _kp, _kd), canHandler(CAN)
{
  baseVelocity = 0;
  baseTorque = 0;
  motorModeOn = false;
}

void MotorHandler::moveMotor(float newPos)
{
  // sending new position to motor library using predefined velocity and torque
  normalSet(canHandler, newPos, baseVelocity, baseTorque);
}

void MotorHandler::exitMotorMode()
{
  byte sendStatus = exitMotormode(canHandler);
  if (sendStatus == CAN_OK)
  {
    motorModeOn = false;
  }
}
void MotorHandler::enterMotorMode()
{
  byte sendStatus = setMotormode(canHandler);
  if (sendStatus == CAN_OK)
  {
    motorModeOn = true;
  }
}

void MotorHandler::setTorqueMode(float tarTor)
{
  // Call the private functions to build the individual packages
  float *torPackage = buildTorquePackage(tarTor);

  //----------------------------------------------------------------------------//
  // Sending data//
  unsigned char buf[8];
  memset(buf, 0, sizeof(buf)); // setting array to 0
  buf[6] = torPackage[0];
  buf[7] = torPackage[1];

  byte sndStat = canHandler.sendMsgBuf(getId(), 0, 8, buf);
}

boolean MotorHandler::resetPosition()
{
  byte sendStatus = setZero(canHandler);

  // check if it was sent succesfully
  if (sendStatus == CAN_OK)
  {
    return true;
  }
  else
  {
    return false;
  }
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

void MotorHandler::clearCANBuffer()
{
  while (canHandler.checkReceive() == CAN_MSGAVAIL)
  {
    INT32U id;
    INT8U ext;
    INT8U len;
    INT8U buf[MAX_CHAR_IN_MESSAGE];
    canHandler.readMsgBuf(&id, &ext, &len, buf);
  }
}

motorResponse MotorHandler::getMotorResponse()
{

  clearCANBuffer();

  // check if motor is in motor mode or not then get response
  if (motorModeOn)
  {
    enterMotorMode();
  }
  else
  {
    exitMotorMode();
  }

  delay(1);

  motorResponse res = handleMotorResponse(canHandler);

  return res;
}

void MotorHandler::printPrettyResponse(motorResponse res)
{
  char outString[64]; // Array to store serial string
  char posString[8];  // Position string
  char velString[8];  // Velocity string
  char currString[8];  // Current string

  

  dtostrf(res.position, 6, 3, posString);
  dtostrf(res.velocity, 6, 3, velString);
  dtostrf(res.current, 6, 3, currString);

  sprintf(outString, "Position: %s  Velocity: %s  Current: %s", posString, velString, currString);
  // Serial.println(msgString2);

  // sprintf(outString, "Position: %.3f  Velocity: %.3f  Current: %.3f", res.position, res.velocity, res.current);
  Serial.println(outString);
}