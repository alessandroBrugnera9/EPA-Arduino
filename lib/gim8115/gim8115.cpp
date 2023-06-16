#include "gim8115.h"
#include <SPI.h> //Serial Peripherial Interface (SPI) Library

unsigned char gim8115::getId() const
{
  return id;
}

int gim8115::setMotormode(MCP_CAN &CAN)
{
  unsigned char len = 0;
  unsigned char buf[8];
  buf[0] = 0xFF;
  buf[1] = 0xFF;
  buf[2] = 0xFF;
  buf[3] = 0xFF;
  buf[4] = 0xFF;
  buf[5] = 0xFF;
  buf[6] = 0xFF;
  buf[7] = 0xFC;

  while (CAN_OK != CAN.sendMsgBuf(id, 0, 8, buf))
  {
    // Serial.println("Entering Motor Mode Failed!");
    // Serial.println("Initializing Motor again");
    delay(100);
  }
  // Serial.println("Motor Mode Enabled!");
  return 1;
}

int gim8115::exitMotormode(MCP_CAN &CAN)
{
  unsigned char len = 0;
  unsigned char buf[8];
  buf[0] = 0xFF;
  buf[1] = 0xFF;
  buf[2] = 0xFF;
  buf[3] = 0xFF;
  buf[4] = 0xFF;
  buf[5] = 0xFF;
  buf[6] = 0xFF;
  buf[7] = 0xFD;

  while (CAN_OK != CAN.sendMsgBuf(id, 0, 8, buf))
  {
    // Serial.println("Exiting Motor Mode Failed!");
    // Serial.println("Exiting Motor again");
    delay(100);
  }
  // Serial.println("Motor Mode Disabled");
  return 1;
}

int gim8115::setZero(MCP_CAN &CAN)
{
  unsigned char len = 0;
  unsigned char buf[8];
  buf[0] = 0xFF;
  buf[1] = 0xFF;
  buf[2] = 0xFF;
  buf[3] = 0xFF;
  buf[4] = 0xFF;
  buf[5] = 0xFF;
  buf[6] = 0xFF;
  buf[7] = 0xFE;

  while (CAN_OK != CAN.sendMsgBuf(id, 0, 8, buf))
  {
    // Serial.println("Zero Setting Failed!");
    // Serial.println("Zero Setting again");
    delay(100);
  }
  // Serial.println("Zero Setted");
  return 1;
}


motorResponse gim8115::handleMotorResponse(MCP_CAN &CAN)
{
  //  Receiving data//
  unsigned char len = 0;
  long unsigned int rxId;
  byte buf_received[6];

  CAN.readMsgBuf(&rxId, &len, buf_received); // CAN BUS reading

  // Collecting bits from CAN_BUS signal
  unsigned int pos_motor = ((buf_received[1] << 8) | buf_received[2]);        // Position reading
  unsigned int vel_motor = (buf_received[3] << 4) | ((buf_received[4]) >> 4); // Velocity reading
  unsigned int cur_motor = ((buf_received[4] & 0xF) << 8) | buf_received[5];  // Current reading

  // Converting to readable data//

  float pos_f = (float)pos_motor;
  pos_f = (pos_f * 191 / 65535) - 95.5;
  float vel_f = (float)vel_motor;
  vel_f = (vel_f * 90 / 4095) - 45;
  float cur_f = (float)cur_motor;
  cur_f = (cur_f * 36 / 4095) - 18;

  // Create an instance of the motorResponse struct and assign the values
  motorResponse response;
  response.position = pos_f;
  response.velocity = vel_f;
  response.current = cur_f;

  // Return the MotorResponse struct
  return response;
}


byte gim8115::normalSet(MCP_CAN &CAN, float tarPos, float tarVel, float tarTor)
{
  // Call the private functions to build the individual packages
  float *posPackage = buildPositionPackage(tarPos);
  float *velPackage = buildVelocityPackage(tarVel);
  float *torPackage = buildTorquePackage(tarTor);

  //----------------------------------------------------------------------------//
  // Sending data//
  unsigned char buf[8];
  buf[0] = posPackage[0];
  buf[1] = posPackage[1];
  buf[2] = velPackage[0];
  buf[3] = velPackage[1] * 16 + kp_16h_hex;
  buf[4] = kp_16l_hex;
  buf[5] = kd_16h_hex;
  buf[6] = kd_16l_hex * 16 + torPackage[0];
  buf[7] = torPackage[1];

  byte sndStat = CAN.sendMsgBuf(id, 0, 8, buf);

  return sndStat;
}

float *gim8115::buildPositionPackage(float tarPos)
{
  float *posPackage = new float[2];

  // Position package building code
  unsigned int pos_b;
  if (tarPos > pos_max)
  {
    tarPos = pos_max;
  }
  else if (tarPos < pos_min)
  {
    tarPos = pos_min;
  }
  tarPos = tarPos + 95.5;
  pos_b = 65535 * tarPos / 191;
  posPackage[0] = pos_b / 256;
  posPackage[1] = pos_b % 256;

  return posPackage;
}

float *gim8115::buildVelocityPackage(float tarVel)
{
  float *velPackage = new float[2];

  // Velocity package building code
  unsigned int vel_b;
  if (tarVel > vel_max)
  {
    tarVel = vel_max;
  }
  else if (tarVel < vel_min)
  {
    tarVel = vel_min;
  }
  tarVel = tarVel + 45;
  vel_b = 4095 * tarVel / 90;
  velPackage[0] = vel_b / 16;
  velPackage[1] = vel_b % 16;

  return velPackage;
}

float *gim8115::buildTorquePackage(float tarTor)
{
  float *torPackage = new float[2];

  // Torque package building code
  unsigned int tor_b;
  if (tarTor > tor_max)
  {
    tarTor = tor_max;
  }
  else if (tarTor < tor_min)
  {
    tarTor = tor_min;
  }
  tarTor = tarTor + 18;
  tor_b = 4095 * tarTor / 36;
  torPackage[0] = tor_b / 256;
  torPackage[1] = tor_b % 256;

  return torPackage;
}
