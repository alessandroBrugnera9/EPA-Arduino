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
    float pos_max = 6.28*20;
    float pos_min = -6.28*20;
    //----------------------------------------//
    float vel_max = 45;
    float vel_min = -45;
    //----------------------------------------//
    float tor_max = 18;
    float tor_min = -18;
    
  public:
    gim8115(int _id, float _kp, float _kd){
      id = (unsigned char) _id;
      //----------------------------------------//
      if (_kp>kp_max)
      {
        _kp = kp_max;
      }
      else if (_kp<kp_min)
      {
        _kp = kp_min;
      }
      kp_b = 4095 * _kp / 500;
      kp_16h = kp_b / 256;
      kp_16l = kp_b % 256;
      kp_16h_hex = kp_16h;
      kp_16l_hex = kp_16l;
      //----------------------------------------//
      if (_kd>kd_max)
      {
        _kd = kd_max;
      }
      else if (_kd<kd_min)
      {
        _kd = kd_min;
      }
      kd_b = 4095 * _kd / 5;
      kd_16h = kd_b / 16;
      kd_16l = kd_b % 16;
      kd_16h_hex = kd_16h;
      kd_16l_hex = kd_16l;

    }

    int setMotormode(MCP_CAN& CAN);
    int exitMotormode(MCP_CAN& CAN);
    int setZero(MCP_CAN& CAN);
    float normalSet(MCP_CAN& CAN, float tarPos, float tarVel, float tarTor);

};
