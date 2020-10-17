//
// Created by Maxim Dobryakov on 18/09/2020.
//

#ifndef AGV_REMOTE_CONTROL_JOYSTICKCONTROLLER_H
#define AGV_REMOTE_CONTROL_JOYSTICKCONTROLLER_H

#include "smoothers/RunningAverageSmoother.h"


#include <driver/adc.h>

class JoystickController {
private:
    static const char *LOG_TAG;

    const adc_bits_width_t ADC_BITS_WIDTH = ADC_WIDTH_BIT_12;

    const adc1_channel_t X_AXIS_ADC_CHANNEL = ADC1_CHANNEL_5;
    const adc1_channel_t Y_AXIS_ADC_CHANNEL = ADC1_CHANNEL_4;

    RunningAverageSmoother xSmoother;
    RunningAverageSmoother ySmoother;

    void initializeADCDriver();
public:
    JoystickController();

    bool calibrateCenter(uint32_t &xCenter, uint32_t &yCenter);
    bool collectPosition(uint32_t &xValue, uint32_t &yValue);

private:
    bool collectRawPosition(uint32_t &xRawValue, uint32_t &yRawValue);
};


#endif //AGV_REMOTE_CONTROL_JOYSTICKCONTROLLER_H
