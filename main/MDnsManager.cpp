//
// Created by Maxim Dobryakov on 15/09/2020.
//

#include "MDnsManager.h"

#include <mdns.h>

MDnsManager::MDnsManager() {
    ESP_ERROR_CHECK(mdns_init());
}

MDnsManager::~MDnsManager() {
    mdns_free();
}

bool MDnsManager::lookupRosMaster(std::string &rosMasterHost, ip4_addr_t &rosMasterIp4, uint16_t &rosMasterPort) const {
    // FYI: Send two queries (_srv and _a) because _ptr can't return IP4 address in most cases.
    // Looks like a bug because response packet from mDNS server has it...
    mdns_result_t *result = nullptr;
    ESP_ERROR_CHECK(mdns_query_srv(
            ROS_MASTER_INSTANCE_NAME.c_str(),
            ROS_MASTER_SERVICE_TYPE.c_str(),
            ROS_MASTER_PROTOCOL.c_str(),
            LOOKUP_TIMEOUT,
            &result));

    if (!result)
        return false;

    rosMasterHost = std::string(result->hostname);
    rosMasterPort = result->port;

    mdns_query_results_free(result);

    esp_ip4_addr_t address;
    esp_err_t queryResult = mdns_query_a(rosMasterHost.c_str(), LOOKUP_TIMEOUT, &address);
    if (queryResult != ESP_OK) // i.e. in case ESP_ERR_NOT_FOUND
        return false;

    rosMasterIp4 = { address.addr };

    return true;
}
