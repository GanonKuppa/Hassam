/*
 * pidController.h
 *
 *  Created on: 2018/11/06
 *      Author: qtfdl
 */

#pragma once

namespace robot_object {

class VelocityTypePidController {
public:
    float Kp;
    float Ki;
    float Kd;
    float e_k0; // 現在偏差
    float e_k1;// 前回偏差
    float e_k2;// 前々回偏差
    float u_k0;// 現在制御量
    float u_k1;// 前回制御量
    float T_s;// 制御周期

    VelocityTypePidController() {
        Kp = 0.0;
        Ki = 1.0;
        Kd = 0.0;
        e_k0 = 0.0;
        e_k1 = 0.0;
        e_k2 = 0.0;
        u_k0 = 0.0;
        u_k1 = 0.0;
        T_s = 0.001;
    }

    void update(float target_, float observed_val_) {
        if(Kp == 0.0) return;

        e_k0 = target_ - observed_val_;
        float delta_u_k = Kp * (e_k0 - e_k1 + T_s / Ki * e_k0 + Kd / T_s * (e_k0 - 2 * e_k1 + e_k2) );
        u_k0 = u_k1 + delta_u_k;

        u_k1 = u_k0;
        e_k2 = e_k1;
        e_k1 = e_k0;

    };

    float getControlVal() {
        if(Kp == 0.0) return 0.0;
        return u_k0;
    }

    void set(float Kp_, float Ki_, float Kd_) {
        Kp = Kp_;
        Ki = Ki_;
        Kd = Kd_;
    }

};

class PidController {
public:
    float Kp;
    float Ki;
    float Kd;
    float int_lim;
    float target;
    float error_p;
    float error_i;
    float error_d;
    float error_p_pre;

    void update(float target_, float observed_val) {
        target = target_;
        error_p = target - observed_val;
        error_i = constrain(error_p+error_i, -int_lim, int_lim);
        error_d = error_p - error_p_pre;
        error_p_pre = error_p;
    };
    float calc() {
        float control_val = Kp * error_p + Ki * error_i + Kd * error_d;
        return control_val;
    };

    void set(float Kp_, float Ki_, float Kd_) {
        Kp = Kp_;
        Ki = Ki_;
        Kd = Kd_;
    }

    void set(float Kp_, float Ki_, float Kd_, float int_lim_) {
        Kp = Kp_;
        Ki = Ki_;
        Kd = Kd_;
        int_lim = int_lim_;
    }

    PidController(float Kp_, float Ki_, float Kd_, float int_lim_) {
        Kp = Kp_;
        Ki = Ki_;
        Kd = Kd_;
        int_lim = int_lim_;
        target = 0.0;
        error_p = 0.0;
        error_i = 0.0;
        error_d = 0.0;
        error_p_pre = 0.0;
    }

    PidController() {
        Kp = 0.0;
        Ki = 0.0;
        Kd = 0.0;
        int_lim = 0.0;
        target = 0.0;
        error_p = 0.0;
        error_i = 0.0;
        error_d = 0.0;
        error_p_pre = 0.0;
    }

};


}
