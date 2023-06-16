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

public:
  /**
   * Constructor for MotorHandler class.
   *
   * @param CAN The MCP_CAN instance used for communication.
   * @param canId The CAN ID of the motor.
   * @param _kp The proportional gain for position control.
   * @param _kd The derivative gain for position control.
   */
  MotorHandler(MCP_CAN &CAN, int canId, float _kp, float _kd);

  /**
   * Move the motor to the specified position using base velocity and torque.
   *
   * @param newPos The desired position to move the motor to.
   */
  void moveMotor(float newPos);

  /**
   * Sends a command to exit motor mode.
   */
  void exitMotorMode();

  /**
   * Sends a command to enter motor mode.
   */
  void enterMotorMode();

  /**
   * Set the motor to torque mode.
   *
   * @param tarTor The target torque value for the motor.
   */
  void setTorqueMode(float tarTor);

  /**
   * Resets the motor position to zero.
   *
   * @return True if the position reset command was sent successfully, false otherwise.
   */
  boolean resetPosition();

  /**
   * Set the motor position, velocity, and torque using the parent class's set function.
   *
   * @param newPos The new position for the motor.
   * @param velocity The velocity for the motor.
   * @param torque The torque for the motor.
   */
  void setPositionFull(float newPos, float velocity, float torque);

  /**
   * Set the base velocity for the moveMotor function.
   *
   * @param baseVelocity The base velocity value.
   */
  void setBaseVelocity(float baseVelocity);

  /**
   * Set the base torque for the moveMotor function.
   *
   * @param baseTorque The base torque value.
   */
  void setBaseTorque(float baseTorque);

  /**
   * @brief Clears the receive buffer by reading and discarding all available messages.
   */
  void clearCANBuffer();

  /**
   * @brief Retrieves the motor response.
   *
   * This function clears the receive buffer, enters or exits motor mode based on the current state,
   * handles the motor response, and returns the position, velocity, and current values.
   *
   * @return A motorResponse struct containing the position, velocity, and current values of the last motor response.
   */
  motorResponse getMotorResponse();

  /**
   * Prints the motor response in a formatted and readable manner.
   *
   * @param res The motorResponse struct containing the position, velocity, and current values.
   */
  void printPrettyResponse(motorResponse res);
};

#endif
