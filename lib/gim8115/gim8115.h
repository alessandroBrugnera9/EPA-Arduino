#include <mcp_can.h>
#include <SPI.h>

#define pi = 3.14

struct motorResponse
{
  float position;
  float velocity;
  float current;
};

class gim8115
{
private:
  unsigned char id;
  //----------------------------------------//
  const float kp_max = 500;
  const float kp_min = 0;
  unsigned char kp_16h_hex;
  unsigned char kp_16l_hex;
  //----------------------------------------//
  const float kd_max = 5;
  const float kd_min = 0;
  unsigned char kd_16h_hex;
  unsigned char kd_16l_hex;
  //----------------------------------------//
  const float pos_max = 6.28 * 20;
  const float pos_min = -6.28 * 20;
  //----------------------------------------//
  const float vel_max = 45;
  const float vel_min = -45;
  //----------------------------------------//
  const float tor_max = 18;
  const float tor_min = -18;

protected:
  /**
   * @brief Builds the position package for the GIM8115 device.
   *
   * This function takes the target position and returns an array of size 2 representing the position package.
   * @param tarPos The target position.
   * @return An array of size 2 representing the position package.
   */
  float *buildPositionPackage(float tarPos);

  /**
   * @brief Builds the velocity package for the GIM8115 device.
   *
   * This function takes the target velocity and returns an array of size 2 representing the velocity package.
   * @param tarVel The target velocity.
   * @return An array of size 2 representing the velocity package.
   */
  float *buildVelocityPackage(float tarVel);

  /**
   * @brief Builds the torque package for the GIM8115 device.
   *
   * This function takes the target torque and returns an array of size 2 representing the torque package.
   * @param tarTor The target torque.
   * @return An array of size 2 representing the torque package.
   */
  float *buildTorquePackage(float tarTor);

public:
  gim8115(int _id, float _kp, float _kd)
  {
    id = static_cast<unsigned char>(_id);
    setKp(_kp);
    setKd(_kd);
  }

  /**
   * Set the proportional gain (Kp).
   *
   * @param _kp The new value for Kp.
   */
  void setKp(float _kp);

  /**
   * Set the derivative gain (Kd).
   *
   * @param _kd The new value for Kd.
   */
  void setKd(float _kd);

  /**
   * Get the ID of the motor in the CAN bus.
   *
   * @return The ID value as an unsigned char.
   */
  unsigned char gim8115::getId() const;

  /**
   * Sends a command to enter motor mode.
   *
   * @param CAN The MCP_CAN object representing the CAN bus.
   * @return The status of the message send operation (CAN_OK (MCP CAN lib) if successful).
   */
  byte gim8115::setMotormode(MCP_CAN &CAN);

  /**
   * Sends a command to exit motor mode.
   *
   * @param CAN The MCP_CAN object representing the CAN bus.
   * @return The status of the message send operation (CAN_OK (MCP CAN lib) if successful).
   */
  byte gim8115::exitMotormode(MCP_CAN &CAN);

  /**
   * Sends a command to set the motor position to zero.
   *
   * @param CAN The MCP_CAN object representing the CAN bus.
   * @return The status of the message send operation (CAN_OK (MCP CAN lib) if successful).
   */
  byte gim8115::setZero(MCP_CAN &CAN);

  /**
   * Reads the motor response data from the MCP_CAN bus and returns the values.
   *
   * @param CAN The MCP_CAN object representing the CAN bus.
   * @return A motorResponse struct containing the position, velocity, and current values of the last motor response.
   */

  /**
   * Reads the motor response data from the MCP_CAN bus and returns the values.
   *
   * @param CAN The MCP_CAN object representing the CAN bus.
   * @return A motorResponse struct containing the position, velocity, and current values of the last motor response.
   */
  motorResponse handleMotorResponse(MCP_CAN &CAN);

  /**
   * @brief Sets the motor to a normal mode with target position, velocity, and torque values.
   *
   * @param CAN The MCP_CAN object representing the CAN bus.
   * @param tarPos The target position value.
   * @param tarVel The target velocity value.
   * @param tarTor The target torque value.
   * @return The status of the message send operation (CAN_OK (MCP CAN lib) if successful).
   */
  byte normalSet(MCP_CAN &CAN, float tarPos, float tarVel, float tarTor);
};
