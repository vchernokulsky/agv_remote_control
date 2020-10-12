//
// Created by Maxim Dobryakov on 16/09/2020.
//

#include "TcpClient.h"


#include <cstring>
#include <esp_log.h>
#include <lwip/sockets.h>
#include <esp_netif.h>

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

    int fcntlResult = fcntl(socketHandle, F_GETFL);
    if (fcntlResult < 0) {
        ESP_LOGE(LOG_TAG, "Unable to get socket flags: errno %s (%d)", std::strerror(errno), errno);

        disconnect();
        return false;
    }
    fcntlResult = fcntlResult | O_NONBLOCK;
    fcntlResult = fcntl(socketHandle, F_SETFL, fcntlResult);
    if (fcntlResult < 0) {
        ESP_LOGE(LOG_TAG, "Unable to set socket flags: errno %s (%d)", std::strerror(errno), errno);

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

    fireOnConnect();

    return true;
}

bool TcpClient::disconnect() {
    ESP_LOGV(LOG_TAG, "Disconnect");

    if (socketHandle < 0) { // already disconnected
        ESP_LOGW(LOG_TAG, "Successfully disconnected already");
        return true;
    }

    bool wasConnected = true;

    int shutdownResult = shutdown(socketHandle, SHUT_RDWR);
    if (shutdownResult < 0) {
        if (errno == ENOTCONN) {
            // If `Socket is not connected` then it's ok and just close socket

            wasConnected = false;
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

    if (wasConnected) {
        fireOnDisconnect();
    }

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

    fd_set rSet;
    FD_ZERO(&rSet);
    FD_SET(socketHandle, &rSet);

    fd_set wSet;
    FD_ZERO(&wSet);
    FD_SET(socketHandle, &wSet);

    struct timeval timeout = { .tv_sec = ConnectionTimeout, .tv_usec = 0 };

    int selectResult = select(FD_SETSIZE, &rSet, &wSet, nullptr, &timeout);
    if (selectResult <= 0) {
        ESP_LOGE(LOG_TAG, "Unable to wait to connection: errno %s (%d)", std::strerror(errno), errno);
        return false;
    }

    // Because `select` return 1 (success) even the connection is not established need to add
    // additional check of connection as recommended here: http://cr.yp.to/docs/connect.html
    //
    // FYI: The check with `getsockopt` described in `UNIX Network Programming. Volume 1` book at
    // `16.4 Nonblocking connect: Daytime Client` section will works the same as check with `getpeername`.
    struct sockaddr_in socketAddress{};
    socklen_t socketAddressLength = sizeof(socketAddress);
    int getPeerNameResult = getpeername(socketHandle, (struct sockaddr *)&socketAddress, &socketAddressLength);
    if (getPeerNameResult != 0) {
        ESP_LOGV(LOG_TAG, "Unable to get information about remote server: errno %s (%d)", std::strerror(errno), errno);
        return false;
    } else {
        esp_ip4_addr addr { .addr = socketAddress.sin_addr.s_addr };
        ESP_LOGD(LOG_TAG, "Connection established with remote server: " IPSTR ":%d", IP2STR(&addr), socketAddress.sin_port);
        return true;
    }
}

void TcpClient::fireOnConnect() const {
    if (onConnect != nullptr)
        onConnect();
}

void TcpClient::fireOnDisconnect() const {
    if (onDisconnect != nullptr)
        onDisconnect();
}
