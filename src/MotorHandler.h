#ifndef MOTOR_HANDLER_H
#define MOTOR_HANDLER_H

#include <gim8115.h>

class MotorHandler : public gim8115
{
private:
  MCP_CAN &canHandler;
  float baseVelocity;
  float baseTorque;

public:
  MotorHandler(MCP_CAN &CAN, int canId, float _kp, float _kd); // Constructor - sets the motor number
  void moveMotor(float newPos);                                // Move the motor to the specified position using base velocity and torque
  // TODO: think about new name to function below to avoid misunderstading with parent class
  void exitMotorMode();                                             // sends command to leave motor mode
  void enterMotorMode();                                            // sends command to enter motor mode
  void setTorqueMode(float tarTor);                                 // Set the motor to torque mode
  void resetPosition();                                             // Reset the motor position to zero
  void setPositionFull(float newPos, float velocity, float torque); // Set normal mode function of the parent class
  void setBaseVelocity(float baseVelocity);                         // define base velocity for movemotor
  void setBaseTorque(float baseTorque);                             // define base torque for movemotor
  void sendLastCommand();                                           // Send last command again to listen to motor parameters
};

#endif
