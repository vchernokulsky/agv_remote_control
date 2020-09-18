//
// Created by Maxim Dobryakov on 18/09/2020.
//

#include "RunningAverageSmoother.h"

RunningAverageSmoother::RunningAverageSmoother() {
}

RunningAverageSmoother::RunningAverageSmoother(double k) : k(k) {
}

RunningAverageSmoother::~RunningAverageSmoother() = default;

uint32_t RunningAverageSmoother::smooth(uint32_t value) {
    if (firstValue) {
        accumulator = value;
        firstValue = false;
    }

    accumulator += (value - accumulator) * k;

    return (uint32_t)accumulator;
}
