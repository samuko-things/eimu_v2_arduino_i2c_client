#include "eimuV2.h"

EIMU_V2::EIMU_V2(int slave_addr)
{
  slaveAddr = slave_addr;
}

float EIMU_V2::readQuat(int pos_no)
{
  get("/quat", pos_no);

  float quat = valA;

  valA = 0.0;
  valB = 0.0;

  return quat;
}

float EIMU_V2::readRPY(int pos_no)
{
  get("/rpy", pos_no);

  float rpy = valA;

  valA = 0.0;
  valB = 0.0;

  return rpy;
}

float EIMU_V2::readRPYVariance(int pos_no)
{
  get("/rpy-var", pos_no);

  float rpy_var = valA;

  valA = 0.0;
  valB = 0.0;

  return rpy_var;
}

float EIMU_V2::readAcc(int pos_no)
{
  get("/acc", pos_no);

  float acc = valA;

  valA = 0.0;
  valB = 0.0;

  return acc;
}

float EIMU_V2::readAccVariance(int pos_no)
{
  get("/acc-var", pos_no);

  float acc_var = valA;

  valA = 0.0;
  valB = 0.0;

  return acc_var;
}

float EIMU_V2::readGyro(int pos_no)
{
  get("/gyro", pos_no);

  float gyro = valA;

  valA = 0.0;
  valB = 0.0;

  return gyro;
}

float EIMU_V2::readGyroVariance(int pos_no)
{
  get("/gyro-var", pos_no);

  float gyro_var = valA;

  valA = 0.0;
  valB = 0.0;

  return gyro_var;
}

bool EIMU_V2::setWorldFrameId(int id=1)
{
  return send("/frame-id", -1, id);
}

int EIMU_V2::getWorldFrameId()
{
  get("/frame-id", -1);

  int id = (int)valA;

  valA = 0.0;
  valB = 0.0;

  return id;
}

float EIMU_V2::getFilterGain()
{
  get("/gain", -1);

  int gain = (int)valA;

  valA = 0.0;
  valB = 0.0;

  return gain;
}

void EIMU_V2::get(String cmd_route, int motor_no)
{
  String msg_buffer = cmd_route;
  msg_buffer += ",";
  msg_buffer += String(motor_no);

  masterSendData(msg_buffer);
  dataMsg = masterReceiveData();

  int indexPos = 0, i = 0;
  do
  {
    indexPos = dataMsg.indexOf(',');
    if (indexPos != -1)
    {
      dataMsgBuffer = dataMsg.substring(0, indexPos);
      dataMsg = dataMsg.substring(indexPos + 1, dataMsg.length());
      dataBuffer[i] = dataMsgBuffer;
      dataMsgBuffer = "";
    }
    else
    {
      if (dataMsg.length() > 0)
        dataBuffer[i] = dataMsg;
    }
    i += 1;
  } while (indexPos >= 0);

  valA = dataBuffer[0].toFloat();
  valB = dataBuffer[1].toFloat();

  dataMsg = "";
  dataMsgBuffer = "";
  dataBuffer[0] = "";
  dataBuffer[1] = "";
}

bool EIMU_V2::send(String cmd_route, int motor_no, float val)
{
  String msg_buffer = cmd_route;
  msg_buffer += ",";
  msg_buffer += String(motor_no);
  msg_buffer += ",";
  msg_buffer += String(val, 3);

  masterSendData(msg_buffer);
  String data = masterReceiveData();
  if (data == "1")
    return true;
  else
    return false;
}

void EIMU_V2::masterSendData(String i2c_msg)
{
  char charArray[i2c_msg.length() + 1];
  i2c_msg.toCharArray(charArray, i2c_msg.length() + 1);

  Wire.beginTransmission(slaveAddr);
  Wire.write(charArray);
  Wire.endTransmission();
}

String EIMU_V2::masterReceiveData()
{
  String i2c_msg = "";
  Wire.requestFrom(slaveAddr, 32);
  while (Wire.available())
  {
    char c = Wire.read();
    i2c_msg += c;
  }
  int indexPos = i2c_msg.indexOf((char)255);
  if (indexPos != -1)
  {
    return i2c_msg.substring(0, indexPos);
  }
  return i2c_msg;
}