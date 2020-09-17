//
// Created by Maxim Dobryakov on 14/09/2020.
//

#ifndef AGV_REMOTE_CONTROL_ESP32HARDWARE_H
#define AGV_REMOTE_CONTROL_ESP32HARDWARE_H

#include "TcpClient.h"

#include <cstdint>

class Esp32Hardware {
private:
    const char* LogTag = "Esp32Hardware";

    TcpClient_t tcpClient = nullptr;

public:
    //FYI: There is no constructor with address/port params (see `init` method)
    //     because NodeHandle call default constructor always... :facepalm:
    virtual ~Esp32Hardware();

    // any initialization code necessary to establish connection to ROS Master
    void init(uint32_t rosMasterAddress, uint16_t rosMasterPort);

    // read a byte from the serial port. -1 = failure
    int read();

    // write data to the connection to ROS
    void write(uint8_t* data, int length);

    // returns milliseconds since start of program
    unsigned long time();
};

#endif //AGV_REMOTE_CONTROL_ESP32HARDWARE_H
