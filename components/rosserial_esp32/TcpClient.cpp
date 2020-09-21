//
// Created by Maxim Dobryakov on 16/09/2020.
//

#include "TcpClient.h"


#include <cstring>
#include <esp_log.h>
#include <lwip/sockets.h>

TcpClient::TcpClient(uint32_t address, uint16_t port)
    : address(address),
      port(port) {
    ESP_LOGV(LOG_TAG, "Initialize TCP client");
}

bool TcpClient::connect() {
    ESP_LOGV(LOG_TAG, "Connect");

    if (socketHandle >= 0) { // already connected
        ESP_LOGW(LOG_TAG, "Successfully connected already");
        return true;
    }

    struct sockaddr_in socketAddress{};
    socketAddress.sin_family = AF_INET;
    socketAddress.sin_addr.s_addr = address;
    socketAddress.sin_port = htons(port);

    socketHandle = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (socketHandle < 0) {
        ESP_LOGE(LOG_TAG, "Unable to create socket: errno %s (%d)", std::strerror(errno), errno);
        return false;
    }

    ESP_LOGD(LOG_TAG, "Socket created, connecting to %s:%d", inet_ntoa(address), port); // inet_ntoa using global buffer

    int fcntlResult = fcntl(socketHandle, F_SETFL, O_NONBLOCK);
    if (fcntlResult < 0) {
        ESP_LOGE(LOG_TAG, "Unable to configure socket: errno %s (%d)", std::strerror(errno), errno);

        disconnect();
        return false;
    }

    int connectResult = ::connect(socketHandle, (struct sockaddr *)&socketAddress, sizeof(struct sockaddr));
    if (connectResult != 0) {
        if (errno == EINPROGRESS) {
            if(!waitConnection()) {
                disconnect();
                return false;
            }
        } else {
            ESP_LOGE(LOG_TAG, "Socket unable to connect: errno %s (%d)", std::strerror(errno), errno);

            disconnect();
            return false;
        }
    }

    ESP_LOGD(LOG_TAG, "Successfully connected");
    return true;
}

bool TcpClient::disconnect() {
    ESP_LOGV(LOG_TAG, "Disconnect");

    if (socketHandle < 0) { // already disconnected
        ESP_LOGW(LOG_TAG, "Successfully disconnected already");
        return true;
    }

    int shutdownResult = shutdown(socketHandle, SHUT_RDWR);
    if (shutdownResult < 0) {
        if (errno == ENOTCONN) {
            // Do Nothing (if `Socket is not connected` then just close socket)
        } else {
            ESP_LOGE(LOG_TAG, "Unable to shutdown socket: errno %s (%d)", std::strerror(errno), errno);
            return false;
        }
    }

    int closeResult = close(socketHandle);
    if (closeResult < 0) {
        ESP_LOGE(LOG_TAG, "Unable to close socket: errno %s (%d)", std::strerror(errno), errno);
        return false;
    }

    socketHandle = -1;

    ESP_LOGD(LOG_TAG, "Successfully disconnected");
    return true;
}

bool TcpClient::read(uint8_t &data) {
    if (socketHandle < 0) {
        if(!connect())
            return false;
    }

    uint8_t buffer[1];
    int readResult = recv(socketHandle, buffer, sizeof(buffer), 0);
    if (readResult < 0) {
        if (errno == EAGAIN) { // Nothing to read from socket
            return false;
        }

        ESP_LOGE(LOG_TAG, "Unable to read from socket: errno %s (%d)", std::strerror(errno), errno);

        disconnect();

        return false;
    }

    data = buffer[0];

    return true;
}

bool TcpClient::write(uint8_t *data, int length) {
    if (socketHandle < 0) {
        if(!connect())
            return false;
    }

    int sendResult = send(socketHandle, data, length, 0);
    if (sendResult < 0) {
        ESP_LOGE(LOG_TAG, "Unable to write to socket: errno %s (%d)", std::strerror(errno), errno);

        disconnect();

        return false;
    }

    return sendResult == length;
}

//FYI: wait async connection (ability to write) as described here:
//     https://www.gnu.org/software/libc/manual/html_node/Waiting-for-I_002fO.html
bool TcpClient::waitConnection() {
    assert(socketHandle >= 0); // socket required

    fd_set set;
    FD_ZERO(&set);
    FD_SET(socketHandle, &set);

    struct timeval timeout = { .tv_sec = ConnectionTimeout, .tv_usec = 0 };

    int selectResult = select(FD_SETSIZE, nullptr, &set, nullptr, &timeout);
    if (selectResult < 0) {
        ESP_LOGE(LOG_TAG, "Unable to wait to connect: errno %s (%d)", std::strerror(errno), errno);
        return false;
    }

    return true;
}
