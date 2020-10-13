//
// Created by Maxim Dobryakov on 08/10/2020.
//

#ifndef AGV_REMOTE_CONTROL_MAPVALUE_H
#define AGV_REMOTE_CONTROL_MAPVALUE_H

#include <cstdint>

int16_t mapFloatToInt16(float floatValue, float minFloatValue, float maxFloatValue, int16_t minInt16Value, int16_t maxInt16Value);
float mapInt16ToFloat(int16_t int16Value, int16_t minInt16Value, int16_t maxInt16Value, float minFloatValue, float maxFloatValue);

int32_t mapFloatToInt32(float floatValue, float minFloatValue, float maxFloatValue, int32_t minInt32Value, int32_t maxInt32Value);
float mapInt32ToFloat(int32_t int32Value, int32_t minInt32Value, int32_t maxInt32Value, float minFloatValue, float maxFloatValue);

#endif //AGV_REMOTE_CONTROL_MAPVALUE_H
