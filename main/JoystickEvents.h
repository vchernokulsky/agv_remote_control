//
// Created by Maxim Dobryakov on 12/09/2020.
//

#ifndef AGV_REMOTE_CONTROL_JOYSTICKEVENTS_H
#define AGV_REMOTE_CONTROL_JOYSTICKEVENTS_H

#include <cstdint>

typedef struct {
    uint32_t xRawValue;
    uint32_t yRawValue;
} JoystickRawEvent;

typedef struct {
    double linearSpeed;
    double angularSpeed;
} JoystickSpeedEvent;

#endif //AGV_REMOTE_CONTROL_JOYSTICKEVENTS_H
