#include "MotorHandler.h"

MotorHandler::MotorHandler(MCP_CAN& CAN, int canId, float _kp, float _kd) : gim8115(canId, _kp, _kd), canHandler(CAN)
{
  baseVelocity = 0;
  baseTorque = 0;
}

void MotorHandler::moveMotor(float newPos)
{
  // sending new position to motor library using predefined velocity and torque
  normalSet(canHandler, newPos, baseVelocity, baseTorque);
}

void MotorHandler::setTorqueMode()
{
  // Code to set motor to torque mode goes here
  // TODO: create torque mode function
  Serial.println("Motor set to torque mode");
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
