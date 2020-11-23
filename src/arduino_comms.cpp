#include "diffdrive_arduino/arduino_comms.h"
#include <sstream>
#include <iostream>
#include <cstdlib>

void ArduinoComms::sendEmptyMsg()
{
    std::string response = sendMsg("\r");
}

void ArduinoComms::readEncoderValues(int &Val1, int &Val2)
{
    std::string response = sendMsg("e\r");

    std::string delimiter = " ";
    size_t delPos = response.find(delimiter);
    std::string token1 = response.substr(0, delPos);
    std::string token2 = response.substr(delPos + delimiter.length());

    Val1 = std::atoi(token1.c_str());
    Val2 = std::atoi(token2.c_str());
}

void ArduinoComms::setMotorValues(int Val1, int Val2)
{
    std::stringstream ss;
    ss << "m " << Val1 << " " << Val2 << "\r";
    sendMsg(ss.str(), false);
}

void ArduinoComms::setPidValues(float Kp, float Kd, float Ki, float Ko)
{
    std::stringstream ss;
    ss << "u " << Kp << ":" << Kd << ":" << Ki << ":" << Ko << "\r";
    sendMsg(ss.str());
}

std::string ArduinoComms::sendMsg(const std::string &MsgToSend, bool PrintOutput)
{
    serialConn.write(MsgToSend);
    std::string response = serialConn.readline();

    if (PrintOutput)
    {
        std::cout << "Sent: " << MsgToSend << std::endl;
        std::cout << "Received: " << response << std::endl;
    }

    return response;
}