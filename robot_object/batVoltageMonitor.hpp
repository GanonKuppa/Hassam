#pragma once

#include "iodefine.h"
#include "stdint.h"
#include "parameterManager.h"
#include "ad.h"
#include <deque>
#include "myUtil.hpp"
#include "communication.h"
#include "sound.h"

using std::deque;

namespace peri = peripheral_RX71M;
using peri:: waitmsec;

namespace robot_object {

class BatVoltageMonitor {

private:
    const uint8_t BUFF_SIZE = 10;
    const float alert_vol = 6.9;
    uint16_t count;
    deque<int16_t> front_buff;
    deque<int16_t> back_buff;

    BatVoltageMonitor() {
        front_bat_vol = 7.4;
        back_bat_vol = 7.4;

        front_average_bat_vol = 7.4;
        back_average_bat_vol = 7.4;
        count = 0;

        for (uint8_t i = 0; i < BUFF_SIZE; i++) {
            front_buff.push_front(0);
            back_buff.push_front(0);
        }

    }

    ~BatVoltageMonitor() {

    }

public:

    float front_bat_vol;
    float back_bat_vol;

    float front_average_bat_vol; //0.5秒間隔で10回サンプルした場合の平均
    float back_average_bat_vol; //

    static BatVoltageMonitor& getInstance() {
         static BatVoltageMonitor instance;
         return instance;
    }

    void update(){
        int16_t front_ad = peri::startAD_AN102();
        int16_t back_ad = peri::startAD_AN101();
        front_bat_vol = 15.1 / 5.1 * (front_ad) * 3.3 / 4096;
        back_bat_vol  = 15.1 / 5.1 * (back_ad) * 3.3 / 4096;

        count ++;
        if(count > 2000) count = 0; //0.25msec割り込み2000回 = 0.5秒
        if(count == 0){
            front_buff.push_front(front_ad);
            back_buff.push_front(back_ad);
            front_buff.pop_back();
            back_buff.pop_back();

            uint16_t front_sum = 0;
            uint16_t back_sum = 0;
            for(auto itr = front_buff.begin(); itr != front_buff.end(); ++itr) {
                   front_sum += *itr;
            }
            for(auto itr = back_buff.begin(); itr != back_buff.end(); ++itr) {
                   back_sum += *itr;
            }

            front_average_bat_vol = 15.1 / 5.1 * (front_sum) * 3.3 / 4096 / BUFF_SIZE;
            back_average_bat_vol = 15.1 / 5.1 * (back_sum) * 3.3 / 4096  / BUFF_SIZE;

        }
    }

    void lowVoltageCheck(){
        if(front_average_bat_vol < alert_vol || back_average_bat_vol < alert_vol )famima();
    }

    void voltageSoundCount(){
        float vol_f = front_bat_vol;
        uint8_t num_1V = uint8_t(vol_f);
        uint8_t num_0_1V = uint8_t((vol_f - float(num_1V)) * 10.0);
        for (int i = 0; i < num_1V; i++) {
            SEA();
            if (i == 4)
                waitmsec(250);
            else
                waitmsec(125);
        }
        waitmsec(125);
        for (int i = 0; i < num_0_1V; i++) {
            SEB();
            if (i == 4)
                waitmsec(250);
            else
                waitmsec(125);
        }
        waitmsec(125);
    }


    void debug(){
        printfAsync("================\n");
        printfAsync("front_now:%f \n", front_bat_vol);
        printfAsync("front_ave %f \n", front_average_bat_vol);
        printfAsync("back_now:%f \n", back_bat_vol);
        printfAsync("back_ave %f \n", back_average_bat_vol);

    }

};

}

