/*-------------------------------------------------------------------------------
Created on Sat Feb 27 2021

@Author: Jihwan Lee
@GIT ID: DGIRobo
@E-mail: fist5678@dgist.ac.kr
@Department: Daegu Gyeongbuk Institute of Science and Technology(DGIST) Undergraduate Course
-------------------------------------------------------------------------------*/
#include <Arduino.h>
#include <mcp_can.h>
#include <SPI.h>
#include "gim8008V3.h"
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
#ifdef ARDUINO_SMD_VARIANT_COMPLANCE
#define SERIAL SerialUSB
#else
#define SERIAL Serial
#endif
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
#define UP A1
#define DOWN A3
#define LEFT A2
#define RIGHT A5
#define CLICK A4
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
#define LED2 8
#define LED3 7
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
// VARIABLES PARA CONTROLAR EL TIEMPO
unsigned long previousMillis = 0;
const long interval = 20;
unsigned long arduinoLoopTime;
unsigned long previousLooptime;
double t;
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
gim8008V3 motor(1, 205, 5);
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
float actPos0 = 0;
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
const int SPI_CS_PIN = 9;
MCP_CAN CAN(SPI_CS_PIN);
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
void setup()
{
  Serial.begin(9600);
  delay(1000);
  while (CAN_OK != CAN.begin(1))
  {
    SERIAL.println("CAN BUS Shield init fail");
    SERIAL.println("Init CAN BUST Shield again");
    delay(2000);
  }
  Serial.println("CAN BUS Shield init ok!");

  pinMode(UP, INPUT);
  pinMode(DOWN, INPUT);
  pinMode(LEFT, INPUT);
  pinMode(RIGHT, INPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);

  digitalWrite(UP, HIGH);
  digitalWrite(DOWN, HIGH);
  digitalWrite(LEFT, HIGH);
  digitalWrite(RIGHT, HIGH);

  digitalWrite(LED2, LOW);
  digitalWrite(LED3, LOW);
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
void loop()
{
  if (Serial.available())
  { // Enquanto a Serial receber dados
    delay(10);
    String comando = "";

    while (Serial.available())
    {                                 // Enquanto receber comandos
      comando += (char)Serial.read(); // Lê os caractéres
    }
    if (comando == "a")
    {
      actPos0 = motor.normalSet(CAN, 2, 2, 2); // Put in order: CANObject, desirePosition(rad), maxVelocity(rad/s), maxTorque(rad/s^2)
      Serial.println(actPos0);
    }
    if (comando == "s")
    {
      actPos0 = motor.normalSet(CAN, 10, 2, 2); // Put in order: CANObject, desirePosition(rad), maxVelocity(rad/s), maxTorque(rad/s^2)
      Serial.println(actPos0);
    }
    if (comando == "f")
    {
      actPos0 = motor.normalSet(CAN, 1000, 2, 2); // Put in order: CANObject, desirePosition(rad), maxVelocity(rad/s), maxTorque(rad/s^2)
      Serial.println(actPos0);
    }
    if (comando == "d")
    {
      Serial.println("oi");
    }
  }
}
