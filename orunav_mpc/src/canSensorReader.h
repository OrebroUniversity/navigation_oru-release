#pragma once

#include "commonDefines.h"
#include "threadData.h"

#include "canlibWrapper.h"


/**
 * @brief Sends commands to a car.
 */
class canSensorReader
{
    public:
        threadReturn readLoop (threadData *);


    private:
        Canlib sw_can;
};
