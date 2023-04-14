/*-------------------------------------------------------------------------------
Created on Sat Feb 27 2021

@Author: Jihwan Lee
@GIT ID: DGIRobo
@E-mail: fist5678@dgist.ac.kr
@Department: Daegu Gyeongbuk Institute of Science and Technology(DGIST) Undergraduate Course
-------------------------------------------------------------------------------*/
#include "gim8008V3.h"

#ifdef ARDUINO_SMD_VARIANT_COMPLANCE
  #define SERIAL SerialUSB
#else
  #define SERIAL Serial
#endif

int gim8008V3::setMotormode(MCP_CAN& CAN){
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

  CAN.sendMsgBuf(id, 0, 8, buf);
  return 1;
}

int gim8008V3::exitMotormode(MCP_CAN& CAN){
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
    
  CAN.sendMsgBuf(id, 0, 8, buf);
  return 1;
}

int gim8008V3::setZero(MCP_CAN& CAN){
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
    
  CAN.sendMsgBuf(id, 0, 8, buf);
  return 1;
}


float gim8008V3::normalSet(MCP_CAN& CAN, float tarPos, float tarVel, float tarTor){
  unsigned int pos_b;
  if (tarPos>pos_max)
  {
    tarPos = pos_max;
  }
  else if (tarPos<pos_min)
  {
    tarPos = pos_min;
  }
  tarPos = tarPos + 95.5;
  pos_b = 65535 * tarPos / 191;
  float pos_16h = pos_b / 256;
  float pos_16l = pos_b % 256;
  unsigned char pos_16h_hex = pos_16h;
  unsigned char pos_16l_hex = pos_16l;
  //----------------------------------------------------------------------------//
  unsigned int vel_b;
  if (tarVel>vel_max)
  {
    tarVel = vel_max;
  }
  else if (tarVel<vel_min)
  {
    tarVel = vel_min;
  }
  tarVel = tarVel + 45;
  vel_b = 4095 * tarVel / 90;
  float vel_16h = vel_b / 16;
  float vel_16l = vel_b % 16;
  unsigned char vel_16h_hex = vel_16h;
  unsigned char vel_16l_hex = vel_16l;
  //----------------------------------------------------------------------------//
  unsigned int tor_b;
  if (tarTor>tor_max)
  {
    tarTor = tor_max;
  }
  else if (tarTor<tor_min)
  {
    tarTor = tor_min;
  }
  tarTor = tarTor + 18;
  tor_b = 4095 * tarTor / 36;
  float tor_16h = tor_b / 256;
  float tor_16l = tor_b % 256;
  unsigned char tor_16h_hex = tor_16h;
  unsigned char tor_16l_hex = tor_16l;
  //----------------------------------------------------------------------------//
  unsigned char len = 0;
  unsigned char buf[8];
  buf[0] = pos_16h;
  buf[1] = pos_16l;
  buf[2] = vel_16h;
  buf[3] = vel_16l*16 + kp_16h_hex;
  buf[4] = kp_16l_hex;
  buf[5] = kd_16h_hex;
  buf[6] = kd_16l_hex*16 + tor_16h_hex;
  buf[7] = tor_16l_hex;

  CAN.sendMsgBuf(id, 0, 8, buf);
  //----------------------------------------------------------------------------//
  float actPos;
  float actVel;
  float actCur;
  if (CAN_MSGAVAIL == CAN.checkReceive())
  {
    CAN.readMsgBuf(&len, buf);
    unsigned long canId = CAN.getCanId();
    float actPos = buf[0]*256 + buf[1];
    float actVel = buf[2]*16 + (buf[3]/16);
    float actCur = (buf[3]%16)*256 + buf[4];
  }

  return actPos;
}

float gim8008V3::setPos(MCP_CAN& CAN, float tarPos){
  SERIAL.println("Not Developed yet.");
  return 0;
}

float gim8008V3::setVel(MCP_CAN& CAN, float tarPos){
  SERIAL.println("Not Developed yet.");
  return 0;
}

float gim8008V3::setTor(MCP_CAN& CAN, float tarPos){
  SERIAL.println("Not Developed yet.");
  return 0;
}
