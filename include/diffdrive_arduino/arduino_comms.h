#include <serial/serial.h>
#include <cstring>

class ArduinoComms
{


public:
  ArduinoComms(const std::string &SerialDevice, int32_t BaudRate, int32_t TimeoutMs)
      : serialConn(SerialDevice, BaudRate, serial::Timeout::simpleTimeout(TimeoutMs))
  {  }


  void sendEmptyMsg();
  void readEncoderValues(int &Val1, int &Val2);
  void setMotorValues(int Val1, int Val2);
  void setPidValues(float Kp, float Kd, float Ki, float Ko);

  bool connected() const { return serialConn.isOpen(); }

  std::string sendMsg(const std::string &MsgToSend, bool PrintOutput = false);


private:
  serial::Serial serialConn;  
};