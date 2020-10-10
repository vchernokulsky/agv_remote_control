//
// Created by Maxim Dobryakov on 18/09/2020.
//

#ifndef AGV_REMOTE_CONTROL_SMOOTHERBASE_H
#define AGV_REMOTE_CONTROL_SMOOTHERBASE_H


#include <cstdint>

class SmootherBase {
public:
    virtual uint32_t smooth(uint32_t value) = 0;
};


#endif //AGV_REMOTE_CONTROL_SMOOTHERBASE_H
