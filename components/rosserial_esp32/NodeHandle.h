//
// Created by Maxim Dobryakov on 16/09/2020.
//

#ifndef AGV_REMOTE_CONTROL_NODEHANDLE_H
#define AGV_REMOTE_CONTROL_NODEHANDLE_H

#include "Esp32Hardware.h"


#include "ros_lib/ros/node_handle.h"

class NodeHandle: public ros::NodeHandle_<Esp32Hardware, 25, 25, 2048, 2048> {
public:
    virtual ~NodeHandle() = default;

    void initNode(uint32_t rosMasterAddress, uint16_t rosMasterPort);
};

#endif //AGV_REMOTE_CONTROL_NODEHANDLE_H
