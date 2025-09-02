#ifndef EIMU_V2_H
#define EIMU_V2_H

#include <Arduino.h>
#include <Wire.h>

class EIMU_V2
{
public:
  EIMU_V2(int);

  float readRPY(int);

  float readQuat(int);

  float readRPYVariance(int);

  float readAcc(int);

  float readAccVariance(int);

  float readGyro(int);

  float readGyroVariance(int);

  bool setWorldFrameId(int);

  int getWorldFrameId();

  float getFilterGain();


private:
  int slaveAddr;
  String dataMsg = "", dataMsgBuffer = "", dataBuffer[2];
  float valA, valB;

  void get(String, int);

  bool send(String, int, float);

  void masterSendData(String);

  String masterReceiveData();
};

#endif