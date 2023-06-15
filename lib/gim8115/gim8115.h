#include <mcp_can.h>
#include <SPI.h>

#define pi = 3.14

class gim8115
{
private:
  unsigned char id;
  //----------------------------------------//
  float kp_max = 500;
  float kp_min = 0;
  unsigned int kp_b;
  float kp_16h;
  float kp_16l;
  unsigned char kp_16h_hex;
  unsigned char kp_16l_hex;
  //----------------------------------------//
  float kd_max = 5;
  float kd_min = 0;
  unsigned int kd_b;
  float kd_16h;
  float kd_16l;
  unsigned char kd_16h_hex;
  unsigned char kd_16l_hex;
  //----------------------------------------//
  float pos_max = 6.28 * 20;
  float pos_min = -6.28 * 20;
  //----------------------------------------//
  float vel_max = 45;
  float vel_min = -45;
  //----------------------------------------//
  float tor_max = 18;
  float tor_min = -18;

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
    id = (unsigned char)_id;
    //----------------------------------------//
    if (_kp > kp_max)
    {
      _kp = kp_max;
    }
    else if (_kp < kp_min)
    {
      _kp = kp_min;
    }
    kp_b = 4095 * _kp / 500;
    kp_16h = kp_b / 256;
    kp_16l = kp_b % 256;
    kp_16h_hex = kp_16h;
    kp_16l_hex = kp_16l;
    //----------------------------------------//
    if (_kd > kd_max)
    {
      _kd = kd_max;
    }
    else if (_kd < kd_min)
    {
      _kd = kd_min;
    }
    kd_b = 4095 * _kd / 5;
    kd_16h = kd_b / 16;
    kd_16l = kd_b % 16;
    kd_16h_hex = kd_16h;
    kd_16l_hex = kd_16l;
  }

  /**
   * @brief Get the ID of the motor in the CAN bus.
   *
   * @return The ID value as an unsigned char.
   */
  unsigned char getId() const;

  int setMotormode(MCP_CAN &CAN);
  int exitMotormode(MCP_CAN &CAN);
  int setZero(MCP_CAN &CAN);
  void handleMotorResponse(MCP_CAN &CAN); //NEED: correct what will retugn

  /**
   * @brief Sets the normal mode for the GIM8115 device.
   *
   * This function sets the position, velocity, and torque for the GIM8115 device and sends the command package.
   * @param tarPos The target position.
   * @param tarVel The target velocity.
   * @param tarTor The target torque.
   * @return 1 if the message was sent successfully, 0 otherwise.
   */
  byte normalSet(MCP_CAN &CAN, float tarPos, float tarVel, float tarTor);
};
