#pragma once

#include <stdint.h>

namespace peripheral_RX71M{

    void initRSPI0(void);
    void initRSPI1(void);
    uint8_t communicate8bitRSPI0(uint8_t transmit);
    uint8_t communicate8bitRSPI1(uint8_t transmit);

    void communicateNbyteRSPI0(uint8_t * send, uint8_t* recv, uint8_t num);
    void communicateNbyteRSPI1(uint8_t * send, uint8_t* recv, uint8_t num);

    void setEnableRSPI0(uint8_t en);
    void setEnableRSPI1(uint8_t en);

}
