#pragma once

#include <stdint.h>
#include <math.h>
#include <myUtil.h>

#include "iodefine.h"
#include "phaseCounting.h"
#include "parameterManager.h"
#include "timer.h"
#include <deque>

using std::deque;
namespace peri = peripheral_RX71M;

namespace robot_object {

class WheelOdometry {
private:
    uint16_t R_ENC_pre;
    uint16_t L_ENC_pre;
    uint16_t R_ENC_now;
    uint16_t L_ENC_now;
    const float BIG_R = 500000.0;
    const float DELTA_T = 0.001;
    const float TIRE_GEAR_NUM = 44.0;
    const float ENC_GEAR_NUM = 11.0;
    const float GEAR_RATIO = 4.0;
    const float ENC_RESOLUTION = 1600.0;
    const float ENC_R_DIR = -1.0; // 機体が進む方向にタイヤを回した場合に位相係数カウント値が増えるか減るか
    const float ENC_L_DIR = -1.0; // 増えるなら 1.0, 減るなら -1.0
    const float PI = 3.14159265358979323846264;

    WheelOdometry() {
        v = 0.0;
        kappa = 0.0;
        ang_v = 0.0;
        r = BIG_R;
        v_R = 0.0;
        v_L = 0.0;

        R_ENC_pre = 0;
        L_ENC_pre = 0;
        R_ENC_now = 0;
        L_ENC_now = 0;

        ab_pos_x = 0.0;
        ab_pos_y = 0.0;
        ab_ang = 0.0;

        rpm_R = 0.0;
        rpm_L = 0.0;

        tire_ang_R = 0.0;
        tire_ang_L = 0.0;
        v_ave = 0.0;

        for (uint8_t i = 0; i < 10; i++) {
            v_list.push_front(0.0);
        }

    }

    ~WheelOdometry() {
    };


public:
    float v;
    float v_R;
    float v_L;
    float rpm_R;
    float rpm_L;
    float ang_v;
    float kappa;
    float r;
    float ab_pos_x;
    float ab_pos_y;
    float ab_ang;
    float tire_ang_R;
    float tire_ang_L;
    float v_ave;
    deque<float> v_list;

    void update() {
        R_ENC_pre = R_ENC_now;
        L_ENC_pre = L_ENC_now;
        R_ENC_now = peri::getCountMTU1();
        L_ENC_now = peri::getCountMTU2();
        int32_t count_diff_R = (int32_t) (R_ENC_now - R_ENC_pre);
        int32_t count_diff_L = (int32_t) (L_ENC_now - L_ENC_pre);
        //オーバーフロー対策
        if (count_diff_R > 32768)
            count_diff_R -= 65536;
        if (count_diff_R < -32768)
            count_diff_R += 65536;
        if (count_diff_L > 32768)
            count_diff_L -= 65536;
        if (count_diff_L < -32768)
            count_diff_L += 65536;

        ParameterManager &pm = ParameterManager::getInstance();
        //エンコーダより計測された速度

        v_R = (ENC_R_DIR * PI * pm.dia_tire / GEAR_RATIO
                * (float) (count_diff_R) / ENC_RESOLUTION) / DELTA_T;
        v_L = (ENC_L_DIR * PI * pm.dia_tire / GEAR_RATIO
                * (float) (count_diff_L) / ENC_RESOLUTION) / DELTA_T;

        v = (v_R + v_L) * 0.5;
        kappa = 2.0 * (v_R - v_L) / (pm.tread * (v_R + v_L));
        float ang_v_rad = (v_R - v_L) / pm.tread; //v * kappa;
        ang_v = RAD2DEG(ang_v_rad);

        if (kappa != 0.0)
            r = 1 / kappa;
        else
            r = BIG_R;

        ab_ang += ang_v * DELTA_T;
        ab_ang = fmodf(ab_ang + 360.0f, 360.0f);
        ab_pos_x += v * cosf(DEG2RAD(ab_ang)) * DELTA_T;
        ab_pos_y += v * sinf(DEG2RAD(ab_ang)) * DELTA_T;

        rpm_R = v_R * 60.0 /(PI * pm.dia_tire);
        rpm_L = v_L * 60.0 /(PI * pm.dia_tire);
        tire_ang_R += count_diff_R / ENC_RESOLUTION / GEAR_RATIO * 360.0;
        tire_ang_L += count_diff_L / ENC_RESOLUTION / GEAR_RATIO * 360.0;
        tire_ang_R = fmod(tire_ang_R+360.0, 360.0);
        tire_ang_L = fmod(tire_ang_L+360.0, 360.0);

        v_list.push_front(v);
        v_list.pop_back();
        v_ave = 0.1 * (v_list[0] + v_list[1] + v_list[2] + v_list[3] + v_list[4] + v_list[5] + v_list[6] + v_list[7] + v_list[8] + v_list[9]);
    }

    float getAveV(bool sb_flag){
        if (sb_flag == true)
            return -v_ave;
        else
            return v_ave;
    }
    float getV(bool sb_flag) {
        if (sb_flag == true)
            return -v;
        else
            return v;
    };

    float getAng_v(bool sb_flag) {
        if (sb_flag == true)
            return ang_v;
        else
            return ang_v;
    };

    float getKappa(bool sb_flag) {
        if (sb_flag == true)
            return -kappa;
        else
            return kappa;
    };

    float getV_R(bool sb_flag) {
        if (sb_flag == true)
            return -v_L;
        else
            return v_R;
    };

    float getV_L(bool sb_flag) {
        if (sb_flag == true)
            return -v_R;
        else
            return v_L;
    };

    float getTireAng_L(){
        return tire_ang_L;
    }

    float getTireAng_R(){
        return tire_ang_R;
    }

    void resetTireAng(){
        tire_ang_R = 0.0;
        tire_ang_L = 0.0;
    }


    static WheelOdometry& getInstance() {
        static WheelOdometry instance;
        return instance;
    }




};

} /* namespace robot_object */

