//
// Created by Maxim Dobryakov on 14/09/2020.
//

#include "Esp32Hardware.h"


#include <cassert>
#include <esp_timer.h>

Esp32Hardware::~Esp32Hardware() {
    assert(tcpClient != nullptr);

    if(!tcpClient->disconnect()) {
        // Do Nothing
    }

    delete tcpClient;

    tcpClient = nullptr;
}

void Esp32Hardware::init(uint32_t rosMasterAddress, uint16_t rosMasterPort) {
    assert(tcpClient == nullptr); // hardware must be initialized only once

    tcpClient = new TcpClient(rosMasterAddress, rosMasterPort);
    if (!tcpClient->connect()) {
        // Do Nothing (will be connected later in read/write methods)
    }
}

int Esp32Hardware::read() {
    assert(tcpClient != nullptr);

    uint8_t byte;
    if(!tcpClient->read(byte)) {
        return -1;
    }

    return byte;
}

void Esp32Hardware::write(uint8_t *data, int length) {
    assert(tcpClient != nullptr);

    if(!tcpClient->write(data, length)) {
        // Do Nothing (because NodeHandle when call this method doesn't handle errors... :facepalm:)
    }
}

unsigned long Esp32Hardware::time() {
    return esp_timer_get_time() / 1000;
}
