//
// Created by Maxim Dobryakov on 16/09/2020.
//

#ifndef AGV_REMOTE_CONTROL_TCPCLIENT_H
#define AGV_REMOTE_CONTROL_TCPCLIENT_H


#include <cstdint>
#include <ctime>

class TcpClient {
private:
    const char* LogTag = "TcpClient";

    const time_t ConnectionTimeout = 3; // sec

    uint32_t address;
    uint16_t port;

    int socketHandle = -1;

public:
    TcpClient(uint32_t address, uint16_t port);

    bool connect();
    bool disconnect();

    bool read(uint8_t &data);
    bool write(uint8_t *data, int length);

private:
    bool waitConnection();
};

typedef TcpClient * TcpClient_t;

#endif //AGV_REMOTE_CONTROL_TCPCLIENT_H
