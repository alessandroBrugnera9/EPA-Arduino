#ifndef MOTOR_HANDLER_H
#define MOTOR_HANDLER_H

#include <gim8115.h>

class MotorHandler : public gim8115
{
private:
  MCP_CAN &canHandler;
  float baseVelocity;
  float baseTorque;
  boolean motorModeOn;

  // NEED: Transform void that have response to byte
  // NEED: implement status of on or off to get position of motor
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
  
  /**
   * Prints the motor response in a formatted and readable manner.
   *
   * @param res The motorResponse struct containing the position, velocity, and current values.
   */
  void MotorHandler::printPrettyResponse(motorResponse res); // receives a motorResponse instance and prints it a human readable way
};

#endif
