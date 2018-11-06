#pragma once

#include <ICM20602.h>
#include <stdint.h>




class Imu {

public:

    float ab_ang;
    float v;

    static Imu& getInstance() {
        static Imu instance;
        return instance;
    }

    void init();
    void update();

private:
    Imu(void) {};
    ~Imu(void) {};
};

