#pragma once

#include <stdint.h>

namespace peripheral_RX71M{

    void initCMTW0();
    void initCMTW1();
    void initTPU0();

    void waitClockCount(uint32_t cCount);
    void waitusec(uint32_t usec);
    void waitmsec(uint32_t msec);
    void startTimeuCount(void);
    unsigned int getTimeuCount(void);
    unsigned int endTimeuCount(void);

    void waitClockCount_sub(uint32_t cCount);
    void waitusec_sub(uint32_t usec);
    void waitmsec_sub(uint32_t msec);
    void startTimeuCount_sub(void);
    unsigned int getTimeuCount_sub(void);
    unsigned int endTimeuCount_sub(void);

    unsigned int getElapsedMsec(void);

}

