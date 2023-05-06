#ifndef MOTOR_HANDLER_H
#define MOTOR_HANDLER_H

#include <gim8115.h>

class MotorHandler : public gim8115
{
private:
  MCP_CAN& canHandler;
  float baseVelocity;
  float baseTorque;

public:
  MotorHandler(MCP_CAN& CAN, int canId, float _kp, float _kd);           // Constructor - sets the motor number
  void moveMotor(float newPos);                                     // Move the motor to the specified position using base velocity and torque
  void setTorqueMode();                                             // Set the motor to torque mode
  void resetPosition();                                             // Reset the motor position to zero
  void setPositionFull(float newPos, float velocity, float torque); // Set normal mode function of the parent class
  void setBaseVelocity(float baseVelocity);                         // define base velocity for movemotor
  void setBaseTorque(float baseTorque);                             // define base torque for movemotor
};

#endif
