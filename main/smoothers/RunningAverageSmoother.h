//
// Created by Maxim Dobryakov on 18/09/2020.
//

#ifndef AGV_REMOTE_CONTROL_RUNNINGAVERAGESMOOTHER_H
#define AGV_REMOTE_CONTROL_RUNNINGAVERAGESMOOTHER_H


#include "SmootherBase.h"

class RunningAverageSmoother: public SmootherBase {
private:
    double k = 0.2;

    bool firstValue = true;
    double accumulator = 0;

public:
    RunningAverageSmoother();
    explicit RunningAverageSmoother(double k);

    virtual ~RunningAverageSmoother();

    uint32_t smooth(uint32_t value) override;
};


#endif //AGV_REMOTE_CONTROL_RUNNINGAVERAGESMOOTHER_H
