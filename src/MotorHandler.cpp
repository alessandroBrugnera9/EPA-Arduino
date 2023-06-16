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
  buf[0] = 0;
  buf[1] = 0;
  buf[2] = 0;
  buf[3] = 0;
  buf[4] = 0;
  buf[5] = 0;
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
  char msgString2[64]; // Array to store serial string
  char print_pos[10];  // Position string
  char print_vel[10];  // Velocity string
  char print_cur[10];  // Current string

  // Transform to string to be printable with snprintf
  snprintf(print_pos, sizeof(print_pos), "%.3f", res.position);
  snprintf(print_vel, sizeof(print_vel), "%.3f", res.velocity);
  snprintf(print_cur, sizeof(print_cur), "%.3f", res.current);

  snprintf(msgString2, sizeof(msgString2), "Position: %s  Velocity: %s  Current: %s", print_pos, print_vel, print_cur);
  Serial.println(msgString2);
}