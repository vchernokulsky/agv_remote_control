//
// Created by Maxim Dobryakov on 16/09/2020.
//

#include "NodeHandle.h"

void NodeHandle::initNode(uint32_t rosMasterAddress, uint16_t rosMasterPort) {
    hardware_.init(rosMasterAddress, rosMasterPort);
    mode_ = 0;
    bytes_ = 0;
    index_ = 0;
    topic_ = 0;
}
