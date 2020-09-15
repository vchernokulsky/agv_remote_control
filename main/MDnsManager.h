//
// Created by Maxim Dobryakov on 15/09/2020.
//

#ifndef AGV_REMOTE_CONTROL_MDNSMANAGER_H
#define AGV_REMOTE_CONTROL_MDNSMANAGER_H


#include <cstdint>
#include <string>
#include <lwip/ip4_addr.h>

class MDnsManager {
    const uint32_t LookupTimeout = 3000; // ms
public:
    const char *RosMasterInstanceName = "ROS Master";
    const char *RosMasterServiceType = "_ros-master";
    const char *RosMasterProtocol = "_tcp";

    MDnsManager();

    virtual ~MDnsManager();

    bool lookupRosMaster(std::string &rosMasterHost, ip4_addr_t &rosMasterIp4, uint16_t &rosMasterPort) const;
};


#endif //AGV_REMOTE_CONTROL_MDNSMANAGER_H
