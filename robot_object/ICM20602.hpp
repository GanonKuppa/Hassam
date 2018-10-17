#pragma once

#include "stdint.h"

namespace robot_object {

class ICM20602{

public:
    volatile int16_t omega_raw[3];
    volatile int16_t acc_raw[3];
    volatile int16_t temp_raw;

    volatile int16_t omega_ref[3];
    volatile int16_t acc_ref[3];

    volatile int16_t omega_c[3];
    volatile int16_t acc_c[3];

    volatile float omega_f[3];
    volatile float acc_f[3];
    volatile float temp_f;

    static ICM20602& getInstance() {
        static ICM20602 instance;
        return instance;
    }

    uint8_t whoAmI(void);
    void init();
    void update();
    void calibOmegaOffset(uint32_t ref_num);
    void calibAccOffset(uint32_t ref_num);


private:
    const float ACC_2g  = 0.000061035156f;      // g/LSB
    const float ACC_4g = 0.000122070312f;       // g/LSB
    const float ACC_8g = 0.000244140625f;       // g/LSB
    const float ACC_16g = 0.000488281250f;      // g/LSB

    const float GYRO_250dps = 0.007633587786f;  // dps/LSB
    const float GYRO_500dps = 0.015267175572f;  // dps/LSB
    const float GYRO_1000dps = 0.030487804878f; // dps/LSB
    const float GYRO_2000dps = 0.060975609756f; // dps/LSB

    const float T_25degC = 0.0030599755201958;  // degC/LSB
    const float RoomTemp_Offset = 25.0;

    const uint16_t REG_WHOAMI = 0x75;
    const uint16_t READ_FLAG = 0x80;

    ICM20602(void) {};
    ~ICM20602(void) {};
    void writeReg(uint8_t adress, uint8_t data);
    uint8_t readReg(uint8_t adress);

};

}
