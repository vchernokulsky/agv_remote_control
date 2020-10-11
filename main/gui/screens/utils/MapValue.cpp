//
// Created by Maxim Dobryakov on 08/10/2020.
//

#include "MapValue.h"

#include <sys/param.h>

template<typename S, typename D>
D map(S sourceValue, S minSourceValue, S maxSourceValue, D minDestValue, D maxDestValue) {
    if (sourceValue < 0) {
        float k = (float)sourceValue / (float)minSourceValue;
        return MAX((D)(k * (float)minDestValue), minDestValue);
    } else if (sourceValue > 0) {
        float k = (float)sourceValue / (float)maxSourceValue;
        return MIN((D)(k * (float)maxDestValue), maxDestValue);
    } else {
        return 0;
    }
}

int16_t mapFloatToInt16(float floatValue, float minFloatValue, float maxFloatValue, int16_t minInt16Value, int16_t maxInt16Value) {
    return map(floatValue, minFloatValue, maxFloatValue, minInt16Value, maxInt16Value);
}

float mapInt16ToFloat(int16_t int16Value, int16_t minInt16Value, int16_t maxInt16Value, float minFloatValue, float maxFloatValue) {
    return map(int16Value, minInt16Value, maxInt16Value, minFloatValue, maxFloatValue);
}
