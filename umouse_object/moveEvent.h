#pragma once

#include <mouse.hpp>
#include <myUtil.hpp>
#include <uart.h>
#include "imu.hpp"
#include <queue>
#include "timer.h"
#include <math.h>
#include "communication.h"
#include "parameterManager.h"
#include "wheelOdometry.hpp"
#include "batVoltageMonitor.hpp"
#include "ICM20602.hpp"

using namespace robot_object;

using std::queue;
namespace peri = peripheral_RX71M;
using peri::getElapsedMsec;
using peri:: waitmsec;

//using namespace Eigen;
//#define EIGEN_DONT_VECTORIZE
//#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

namespace robot_object {

class SecondOrderEuler {
public:
    float x;
    float v;
    float a;
    float delta_t;
    void update() {
        v += delta_t * a;
        x += delta_t * v;
    }
    void set(float a_, float v_, float x_) {
        a = a_;
        v = v_;
        x = x_;
    }

    void set(float a_, float v_, float x_, float delta_t_) {
        a = a_;
        v = v_;
        x = x_;
        delta_t = delta_t_;
    }

    SecondOrderEuler(float a_0, float v_0, float x_0, float delta_t_) {
        delta_t = delta_t_;
        a = a_0;
        v = v_0;
        x = x_0;
    }

    SecondOrderEuler() {
        delta_t = 0.0;
        a = 0.0;
        v = 0.0;
        x = 0.0;
    }

};

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
        e_k1 = e_k0;
        e_k2 = e_k1;
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

class WallController : public VelocityTypePidController {

public:
    using VelocityTypePidController::VelocityTypePidController;

    void update(WallSensor& ws) {

        float error;

        int16_t e_fr = 0;
        int16_t e_fl = 0;
        int16_t e_br = 0;
        int16_t e_bl = 0;
        uint8_t valid_error_num = 0;

        if(ws.isRight_for_ctrl() == 1) e_fr = -(ws.right() - ws.center_r() );
        if(ws.isLeft_for_ctrl() == 1) e_fl = +(ws.left() - ws.center_l() );
        //if(ws.isRight_for_ctrl_b() == 1) e_br = +(ws.right_b() - ws.center_r_b());
        //if(ws.isLeft_for_ctrl_b()  == 1) e_bl = -(ws.left_b()  - ws.center_l_b());



        //if(ws.ahead() > 900){ e_fr = 0; e_fl=0;}
        //if(ws.ahead_b() > 900){ e_br = 0; e_bl=0;}

        if(e_fr != 0) valid_error_num ++;
        if(e_fl != 0) valid_error_num ++;
        //if(e_br != 0) valid_error_num ++;
        //if(e_bl != 0) valid_error_num ++;

        if(valid_error_num != 0) error = (float)(e_fr + e_fl + e_br + e_bl) / (float)(valid_error_num);
        else error = 0.0f;

        UMouse &m = UMouse::getInstance();
        e_k0 = -error;

        float delta_u_k = Kp * (e_k0 - e_k1 + T_s / Ki * e_k0 + Kd / T_s * (e_k0 - 2 * e_k1 + e_k2) );
        u_k0 = u_k1 + delta_u_k;

        u_k1 = u_k0;
        e_k1 = e_k0;
        e_k2 = e_k1;

    };

};

class BaseMoveEvent {

protected:
    VelocityTypePidController v_ctrl;
    VelocityTypePidController ang_v_ctrl;
    VelocityTypePidController ang_ctrl;
    WallController wall_ctrl;

    SecondOrderEuler trans_t;
    SecondOrderEuler rot_t;

    float accum_ang;
    float accum_x;
    uint32_t count;
    const float DELTA_T = 0.001;
    const float PI = 3.14159265358979323846264;
    const float TIRE_GEAR_NUM = 44.0;
    const float ENC_GEAR_NUM = 9.0;
    const float GEAR_RATIO = 4.888889;

    BaseMoveEvent() {
        accum_ang = 0.0;
        accum_x = 0.0;
        count = 0;
    }

    void updateMousePidErrorVal() {
        UMouse &m = UMouse::getInstance();

        m.t_ang_a = rot_t.a;
        m.t_ang_v = rot_t.v;
        m.t_ang = rot_t.x;
        m.accum_ang = accum_ang;

        m.t_a = trans_t.a;
        m.t_v = trans_t.v;
        m.t_x = trans_t.x;
        m.accum_x = accum_x;

    }

public:
    virtual bool isEnd()=0;

    Vector2f calcFFDuty(float target_a, float target_v,
            float target_ang_a, float target_ang_v) {
        ParameterManager &pm = ParameterManager::getInstance();
        UMouse &mouse = UMouse::getInstance();
        Vector2f duty(0.0f, 0.0f);
        BatVoltageMonitor &batVolMon = BatVoltageMonitor::getInstance();

        //モータとモータドライバの抵抗値[Ω]
        float R_R;
        float R_L;
        float offset_torque_R;
        float offset_torque_L;
        float torque_scaler_R;
        float torque_scaler_L;
        float K_T_R;
        float K_T_L;
        float K_E_R;
        float K_E_L;
        float Vcc_L;
        float Vcc_R;

        if(mouse.switch_back == false) {
            R_R = pm.circuit_res_right;
            R_L = pm.circuit_res_left;
            offset_torque_R = pm.offset_torque_right;
            offset_torque_L = pm.offset_torque_left;
            torque_scaler_R = pm.torque_scaler_right;
            torque_scaler_L = pm.torque_scaler_left;
            K_T_R = pm.K_T_right;
            K_T_L = pm.K_T_left;
            K_E_R = (2 * PI) / 60 * K_T_R;
            K_E_L = (2 * PI) / 60 * K_T_L;
            Vcc_R = batVolMon.back_bat_vol;
            Vcc_L = batVolMon.front_bat_vol;
        } else {
            R_R = pm.circuit_res_left;
            R_L = pm.circuit_res_right;
            offset_torque_R = pm.offset_torque_left;
            offset_torque_L = pm.offset_torque_right;
            torque_scaler_R = pm.torque_scaler_left;
            torque_scaler_L = pm.torque_scaler_right;
            K_T_R = pm.K_T_left;
            K_T_L = pm.K_T_right;
            K_E_R = (2 * PI) / 60 * K_T_R;
            K_E_L = (2 * PI) / 60 * K_T_L;
            Vcc_R = batVolMon.front_bat_vol;
            Vcc_L = batVolMon.back_bat_vol;
        }

        //左右輪の速度
        float v_L = target_v - pm.tread * PI * target_ang_v/360.0;
        float v_R = target_v + pm.tread * PI * target_ang_v/360.0;

        //必要な力(回転成分)
        float F_rotation = (pm.inertia * DEG2RAD(target_ang_a))/ pm.tread;

        //必要な力(直進成分)[N]
        float F_forward = pm.mass*target_a/2.0;

        //左右モーターの回転数
        float rpm_L = v_L * 60.0 /(PI * pm.dia_tire);
        float rpm_R = v_R * 60.0 /(PI * pm.dia_tire);

        //必要なトルク
        float torque_L = (F_forward * (0.5 * pm.dia_tire/GEAR_RATIO)) - (F_rotation)*(0.5 * pm.dia_tire/GEAR_RATIO);
        float torque_R = (F_forward * (0.5 * pm.dia_tire/GEAR_RATIO)) + (F_rotation)*(0.5 * pm.dia_tire/GEAR_RATIO);
        torque_L = torque_L * torque_scaler_L + SIGN(rpm_L) * offset_torque_L;
        torque_R = torque_R * torque_scaler_R + SIGN(rpm_R) * offset_torque_R;

        duty.x = (R_L*torque_L / K_T_L + K_E_L * rpm_L) / Vcc_L;
        duty.y = (R_R*torque_R / K_T_R + K_E_R * rpm_R) / Vcc_R;
        return duty;
    };

    Vector2f calcFBDuty(float v_FB, float ang_v_FB) {
        ParameterManager &pm = ParameterManager::getInstance();
        UMouse &mouse = UMouse::getInstance();
        Vector2f duty(0.0f, 0.0f);

        WheelOdometry &wodo = WheelOdometry::getInstance();
        BatVoltageMonitor &batVolMon = BatVoltageMonitor::getInstance();

        //モータとモータドライバの抵抗値[Ω]
        float R_R;
        float R_L;
        float offset_torque_R;
        float offset_torque_L;
        float torque_scaler_R;
        float torque_scaler_L;
        float K_T_R;
        float K_T_L;
        float K_E_R;
        float K_E_L;
        float Vcc_L;
        float Vcc_R;

        if(mouse.switch_back == false) {
            R_R = pm.circuit_res_right;
            R_L = pm.circuit_res_left;
            offset_torque_R = pm.offset_torque_right;
            offset_torque_L = pm.offset_torque_left;
            torque_scaler_R = pm.torque_scaler_right;
            torque_scaler_L = pm.torque_scaler_left;
            K_T_R = pm.K_T_right;
            K_T_L = pm.K_T_left;
            Vcc_R = batVolMon.back_bat_vol;
            Vcc_L = batVolMon.front_bat_vol;
        } else {
            R_R = pm.circuit_res_left;
            R_L = pm.circuit_res_right;
            offset_torque_R = pm.offset_torque_left;
            offset_torque_L = pm.offset_torque_right;
            torque_scaler_R = pm.torque_scaler_left;
            torque_scaler_L = pm.torque_scaler_right;
            K_T_R = pm.K_T_left;
            K_T_L = pm.K_T_right;
            Vcc_R = batVolMon.front_bat_vol;
            Vcc_L = batVolMon.back_bat_vol;
        }
        //必要なトルク
        float torque_L = (v_FB - ang_v_FB) * torque_scaler_L;
        float torque_R = (v_FB + ang_v_FB) * torque_scaler_R;

        duty.x = (R_L*torque_L / K_T_L) / Vcc_L;
        duty.y = (R_R*torque_R / K_T_R) / Vcc_R;

        return duty;
    };

    virtual void updateTarget() {
        trans_t.update();
        rot_t.update();
    }

    virtual Vector2f calcDuty() {
        Vector2f duty(0.0,0.0);
        UMouse &m = UMouse::getInstance();
        ParameterManager &pm = ParameterManager::getInstance();
        WallSensor &ws = WallSensor::getInstance();
        WheelOdometry &wodo = WheelOdometry::getInstance();
        ICM20602 &imu = ICM20602::getInstance();

        //ターンの開始時実行処理
        if(count == 0) {
            //ang_v_ctrl.error_i = m.ang_v_I / ang_v_ctrl.Ki;
            //v_ctrl.error_i = m.v_I / v_ctrl.Ki;
        }

        //ターゲットの積分
        updateTarget();

        //実角度,実距離の積分
        accum_ang += imu.omega_f[2] * DELTA_T;
        accum_x += wodo.getV(m.switch_back) * DELTA_T;

        //角度, 壁制御量の算出
        wall_ctrl.update(ws);
        ang_ctrl.update(rot_t.x, accum_ang);
        float FB_ang = ang_ctrl.getControlVal();
        float FB_wall = wall_ctrl.getControlVal();

        //角速度, 速度制御量の算出
        float target_v = trans_t.v;
        float target_ang_v = rot_t.v + FB_ang + FB_wall;
        ang_v_ctrl.update(target_ang_v, imu.omega_f[2]);//imu.omega_f[2] wodo.getAng_v(m.switch_back)
        v_ctrl.update(target_v, wodo.getAveV(m.switch_back));

        //制御量をdutyに変換
        float target_a = trans_t.a;
        float target_ang_a = rot_t.a;

        Vector2f dutyFF(0.0f, 0.0f);
        dutyFF = calcFFDuty(target_a, target_v, target_ang_a, target_ang_v);

        float FB_v = v_ctrl.getControlVal();
        float FB_ang_v = ang_v_ctrl.getControlVal();
        Vector2f dutyFB(0.0f, 0.0f);
        dutyFB = calcFBDuty(FB_v, FB_ang_v);

        duty = dutyFF + dutyFB;

        //計算した制御量をmouseにセット(制御量のグラフ描画用)
        updateMousePidErrorVal();

        count ++;

        return duty;
    };

    virtual ~BaseMoveEvent() {};

};

class Trape : public BaseMoveEvent {

public:
    float x_acc;
    float x_bre;
    float x_bre_now;
    float x;
    float v_max;
    float v_0;
    float v_end;
    float a;

    Trape(float x_, float v_max_, float v_0_, float v_end_, float a_, bool wall_ctrl_flag) {
        ParameterManager &pm = ParameterManager::getInstance();
        //変数初期化
        x = x_;
        v_max = v_max_;
        v_0 = v_0_;
        v_end = v_end_;
        a = a_;
        x_bre_now = 0.0;

        accum_ang = 0.0;
        accum_x = 0.0;

        if(a != 0.0) {
            x_acc = ( (v_max * v_max) - (v_0 * v_0) ) / (2.0*a);
            x_bre = ( (v_max * v_max) - (v_end * v_end) ) / (2.0*a);
        }
        else {
            x_acc = 0.0;
            x_bre = 0.0;
        }

        //制御の初期化
        ang_v_ctrl.set(pm.straight_ang_v_P, pm.straight_ang_v_I, pm.straight_ang_v_D);
        ang_ctrl.set(pm.straight_ang_P, pm.straight_ang_I, pm.straight_ang_D);
        v_ctrl.set(pm.straight_v_P, pm.straight_v_I, pm.straight_v_D);

        if(wall_ctrl_flag == false)wall_ctrl.set(0.0, 1.0, 0.0);
        else wall_ctrl.set(pm.wall_P, pm.wall_I, pm.wall_D);
        //ターゲットの初期化
        trans_t.set(a, v_0, 0.0, DELTA_T);
        rot_t.set(0.0, 0.0, 0.0, DELTA_T);

        //printfAsync("In Trape\n");
    };

    bool isEnd() {

        //終了条件
        if(v_end == 0.0) {
            if(trans_t.v < 0.0 || trans_t.x > x) {
                //printfAsync("Out Trape\n");
                return true;
            }
        }
        else {
            if(accum_x >= x) {
                //printfAsync("Out Trape\n");
                return true;
            }
        }

        //  速度がv_endになるように台形加速
        if( (x_acc + x_bre) < x ) {
            if ( trans_t.x < x_acc) {
                trans_t.a = a;
            }
            else if( trans_t.x < x-x_bre) {
                trans_t.a = 0.0;
                trans_t.v = v_max;
            }
            else if( trans_t.x < x) {
                trans_t.a = -1.0 * a;
            }
            else if( trans_t.x >= x) {
                trans_t.a = 0.0;
                trans_t.v = v_end;
            }
        }
        //加速度が低くv_endに到達できない場合
        else {
            if(a != 0.0) {
                x_bre_now = ( (trans_t.v * trans_t.v) - (v_end * v_end) ) / (2.0*a);
            }
            else {
                x_bre_now = 0.0;
            }

            if( (x_bre_now >= (x - trans_t.x)) && (trans_t.a > 0.0) ) {
                trans_t.a = (-1.0)*a;
            }
        }

        WallSensor& ws = WallSensor::getInstance();
        ParameterManager &pm = ParameterManager::getInstance();
        //if(ws.ahead.at(0)>pm.collision_thr_ahead) return true;

        return false;
    };

};

class Stop : public BaseMoveEvent {

public:

    Stop(uint32_t stop_ms) {
        ParameterManager &pm = ParameterManager::getInstance();
        count_end = stop_ms;
        //制御の初期化
        ang_v_ctrl.set(pm.straight_ang_v_P, pm.straight_ang_v_I, pm.straight_ang_v_D);
        ang_ctrl.set(pm.straight_ang_P, pm.straight_ang_I, pm.straight_ang_D);
        v_ctrl.set(pm.straight_v_P, pm.straight_v_I, pm.straight_v_D);
        wall_ctrl.set(0.0, 1.0, 0.0);
        //ターゲットの初期化
        trans_t.set(0.0, 0.0, 0.0, DELTA_T);
        rot_t.set(0.0, 0.0, 0.0, DELTA_T);

        //printfAsync("In Stop\n");
    }

    bool isEnd() {
        count ++;
        if(count < count_end) return false;
        else {
            //printfAsync("Out Stop\n");
            return true;
        }
    };

private:
    uint32_t count_end;

};

class SwitchBack : public BaseMoveEvent {

public:

    SwitchBack() {
        ParameterManager &pm = ParameterManager::getInstance();

        count_end = 50;

        //制御の初期化
        ang_v_ctrl.set(0.0, 0.0, 0.0);
        ang_ctrl.set(0.0, 0.0, 0.0);
        v_ctrl.set(0.0, 0.0, 0.0);
        wall_ctrl.set(0.0, 1.0, 0.0);
        //ターゲットの初期化
        trans_t.set(0.0, 0.0, 0.0, DELTA_T);
        rot_t.set(0.0, 0.0, 0.0, DELTA_T);

        //printfAsync("In SWITCH BACK\n");
    }

    bool isEnd() {
        count ++;
        if(count < count_end) return false;
        else {
            //printfAsync("Out SWITCH BACK\n");
            UMouse &m = UMouse::getInstance();
            m.switch_back = 1 - m.switch_back;
            return true;
        }
    };

private:
    uint32_t count = 0;
    uint32_t count_end;

};

class PivotTurn : public BaseMoveEvent {

public:

    float end_ang;
    float rot_a;
    float abs_min_rot_v;

    PivotTurn(float ang) {
        ParameterManager &pm = ParameterManager::getInstance();

        //変数初期化
        end_ang = ang;
        rot_a = SIGN(ang)* 1000.0;
        accum_ang = 0.0;
        accum_x = 0.0;
        abs_min_rot_v = 15.0;
        //printfAsync("PivotTurn v_ctrl\n ");

        //各種制御の初期化
        ang_v_ctrl.set(pm.pivot_ang_v_P, pm.pivot_ang_v_I, pm.pivot_ang_v_D);
        ang_ctrl.set(pm.pivot_ang_P, pm.pivot_ang_I, pm.pivot_ang_D);
        v_ctrl.set(pm.pivot_v_P, pm.pivot_v_I, pm.pivot_v_D);
        wall_ctrl.set(0.0, 1.0, 0.0);

        //ターゲットの初期化
        trans_t.set(0.0, 0.0, 0.0, DELTA_T);
        rot_t.set(rot_a, 0.0, 0.0, DELTA_T);

    };

    bool isEnd() {
        if(end_ang == 0.0) return true;

        if(ABS(rot_t.x) < ABS(end_ang)*1.0/2.0) {
            rot_t.a = rot_a;
        }
        else if(ABS(rot_t.x) < ABS(end_ang)) {
            rot_t.a = - rot_a;
            if( ABS(rot_t.v) <= abs_min_rot_v || SIGN(rot_t.v) != SIGN(end_ang)) {
                rot_t.v = SIGN(end_ang)* abs_min_rot_v;
                rot_t.a = 0.0;
            }
        }
        else {
            rot_t.v = SIGN(end_ang)* abs_min_rot_v;
            rot_t.a = 0.0;
            if(ABS(accum_ang) > ABS(end_ang)) return true;
        }

        return false;
    };

};

class Trace : public BaseMoveEvent {

public:

    float end_ang;
    float rot_a;
    float abs_min_rot_v;
    float trace_v;
    float trace_a;
    PidController line_trace;

    float calc_val = 0.0;
    float calc_val_pre = 0.0;
    uint8_t count_cross = 0;

    Trace() {
        ParameterManager &pm = ParameterManager::getInstance();

        //変数初期化
        //trace_v = pm.tracer_v;
        trace_a = 2.0;
        //printfAsync("trace\n ");

        //各種制御の初期化
        //ang_v_ctrl.set(pm.rot_P, pm.rot_I, pm.rot_D, pm.rot_LI);
        //ang_ctrl.set(0.0, 0.0, 0.0, 0.0);
        //v_ctrl.set(pm.straight_P, pm.straight_I, pm.straight_D, pm.straight_LI);
        //wall_ctrl.set(0.0, 0.0, 0.0);
        //line_trace.set(pm.trace_P, pm.trace_I, pm.trace_D, pm.trace_LI);

        //ターゲットの初期化
        trans_t.set(0.0, trace_a, 0.0, DELTA_T);
        rot_t.set(0.0, 0.0, 0.0, DELTA_T);

    };

    bool isEnd() {
        BackWallSensor& bws = BackWallSensor::getInstance();
        WallSensor& ws = WallSensor::getInstance();
        if(trans_t.v > trace_v) {
            trans_t.v = trace_v;
            trans_t.a = 0.0;
        } else {

            ParameterManager &pm = ParameterManager::getInstance();
            bool l, c, r, last_l, last_r;
            if (ws.left() > 650) l = 1;
            else l = 0;
            if (ws.ahead() > 1000) c = 1;
            else c = 0;
            if(ws.right()> 650)r = 1;
            else r = 0;

            if(count_cross == 0) {
                if(l == 0 && c == 0 && r == 0) {
                    if(calc_val_pre > 0.0) {
                        //calc_val = pm.tracer_gain * 3.5;
                    } else if(calc_val_pre < 0.0) {
                        //calc_val = pm.tracer_gain * -3.5;
                    } else {
                        calc_val = 0.0;
                    }
                }
                else if(l == 1 && c == 0 && r == 0) {
                    //calc_val = pm.tracer_gain * 2.0;
                }
                else if(l == 1 && c == 1 && r == 0) {
                    //calc_val = pm.tracer_gain * 1.0;
                }
                else if(l == 0 && c == 1 && r == 0) {
                    calc_val = 0.001;
                }
                else if(l == 0 && c == 1 && r == 1) {
                    //calc_val = pm.tracer_gain * -1.0;
                }
                else if(l == 0 && c == 0 && r == 1) {
                    //calc_val = pm.tracer_gain * -2.0;
                }
                else if(l == 1 && c == 1 && r == 1) {
                    calc_val = 0.0;
                    count_cross = 50;
                }
            }
            else {
                calc_val = 0.0;
            }

            rot_t.v = calc_val;
            calc_val_pre = calc_val;
            if(count_cross != 0) count_cross--;
            else count_cross = 0;

        }

        if(count > 10000 && (bws.right.at(0) >550 || bws.left.at(0) > 350 ))return true;
        else return false;
    };

};

class SlalomWithKappa : public BaseMoveEvent{

public:
    float rot_dir;
    float d_pre;
    float d_fol;
    const float *kappa_list;
    uint16_t kappa_list_len;
    float delta_s_len;
    float s_path_len;

    void setKappaParam(const float* kappa_list_, uint16_t kappa_list_len_, float delta_s_len_, float s_path_len_ , float d_pre_, float d_fol_){
        kappa_list = kappa_list_;
        kappa_list_len = kappa_list_len_;
        delta_s_len = delta_s_len_;
        s_path_len = s_path_len_;
        d_pre = d_pre_;
        d_fol = d_fol_;
    }

    SlalomWithKappa(){
    }

    SlalomWithKappa(float v, float dir) {
        ParameterManager &pm = ParameterManager::getInstance();

        //変数初期化
        accum_ang = 0.0;
        accum_x = 0.0;
        //printfAsync("In slalom classic 90deg\n ");

        rot_dir = SIGN(dir);
        //各種制御の初期化
        ang_v_ctrl.set(pm.slalom_ang_v_P, pm.slalom_ang_v_I, pm.slalom_ang_v_D);
        ang_ctrl.set(pm.slalom_ang_P, pm.slalom_ang_I, pm.slalom_ang_D);
        v_ctrl.set(pm.slalom_v_P, pm.slalom_v_I, pm.slalom_v_D);
        wall_ctrl.set(0.0, 1.0, 0.0);

        //ターゲットの初期化
        trans_t.set(0.0, v, 0.0, DELTA_T);
        rot_t.set(0.0, 0.0, 0.0, DELTA_T);

    };

    SlalomWithKappa(float v, float dir, bool search_flag) {
        ParameterManager &pm = ParameterManager::getInstance();

        //変数初期化
        accum_ang = 0.0;
        accum_x = 0.0;
        if(pm.half_flag == 0){
            if(search_flag == true) {
                d_pre += 0.01;
                d_fol -= 0.01;
            }
        }
        else{
            if(search_flag == true) {
                d_pre += 0.005;
                d_fol -= 0.005;
            }
        }
        //printfAsync("In slalom classic 90deg\n ");

        rot_dir = SIGN(dir);
        //各種制御の初期化
        ang_v_ctrl.set(pm.slalom_ang_v_P, pm.slalom_ang_v_I, pm.slalom_ang_v_D);
        ang_ctrl.set(pm.slalom_ang_P, pm.slalom_ang_I, pm.slalom_ang_D);
        v_ctrl.set(pm.slalom_v_P, pm.slalom_v_I, pm.slalom_v_D);
        wall_ctrl.set(0.0, 1.0, 0.0);

        //ターゲットの初期化
        trans_t.set(0.0, v, 0.0, DELTA_T);
        rot_t.set(0.0, 0.0, 0.0, DELTA_T);

    };

    bool isEnd() {
        printfAsync("e\n ");

        if(accum_x > s_path_len + d_pre + d_fol) {
            //printfAsync("Out slalom classic 90deg\n ");
            return true;
        }
        else return false;
    };

    void updateTarget(){
        trans_t.update();
        if(accum_x > d_pre && accum_x < d_pre + s_path_len) {
            uint16_t index = uint16_t( (trans_t.x - d_pre) /delta_s_len);

            if(index <= 0) {
                index = 0;
                rot_t.a = 0.0;
                rot_t.v = 0.0;
                rot_t.x += rot_t.v * DELTA_T;
            }
            else if(index >= kappa_list_len - 1) {
                rot_t.a = 0.0;
                rot_t.v = 0.0;
                rot_t.x += rot_t.v * DELTA_T;
            }
            else {
                float rot_t_v_pre = rot_t.v;
                rot_t.v = RAD2DEG(rot_dir * kappa_list[index] * trans_t.v);
                rot_t.a = (rot_t.v - rot_t_v_pre)/DELTA_T;
                rot_t.x += rot_t.v * DELTA_T;
            }
        }
        else {
            rot_t.v = 0.0;
            rot_t.a = 0.0;
        }
    }
};

#include "kappa.h"

class Slalom90deg : public SlalomWithKappa{
public:
    Slalom90deg(float v, float dir, bool search_flag){
        setKappaParam(&kappa_90deg_classic[0],
                kappa_len_90deg_classic,
                delta_s_len_90deg_classic,
                path_len_90deg_classic,
                d_pre_90deg_classic,
                d_fol_90deg_classic);
        SlalomWithKappa(v, dir, search_flag);
    };

    Slalom90deg(float v, float dir){
        setKappaParam(&kappa_90deg_classic[0],
                kappa_len_90deg_classic,
                delta_s_len_90deg_classic,
                path_len_90deg_classic,
                d_pre_90deg_classic,
                d_fol_90deg_classic);

        SlalomWithKappa(v, dir);
    };
};

class HF_Slalom90deg : public SlalomWithKappa{
public:
    HF_Slalom90deg(float v, float dir, bool search_flag){
        setKappaParam(&kappa_90deg_half[0],
                kappa_len_90deg_half,
                delta_s_len_90deg_half,
                path_len_90deg_half,
                d_pre_90deg_half,
                d_fol_90deg_half);
        SlalomWithKappa(v, dir, search_flag);
    };

    HF_Slalom90deg(float v, float dir){
        setKappaParam(&kappa_90deg_half[0],
                kappa_len_90deg_half,
                delta_s_len_90deg_half,
                path_len_90deg_half,
                d_pre_90deg_half,
                d_fol_90deg_half);

        SlalomWithKappa(v, dir);
    };
};



class Slalom_classic_90deg : public BaseMoveEvent {

private:
#include "kappa.h"

public:

    float rot_dir;
    float d_pre;
    float d_fol;


    Slalom_classic_90deg(float v, float dir) {
        ParameterManager &pm = ParameterManager::getInstance();

        //変数初期化
        accum_ang = 0.0;
        accum_x = 0.0;
        d_pre = 0.03031937846141211;
        d_fol = 0.03029937846158079;
        //printfAsync("In slalom classic 90deg\n ");

        rot_dir = SIGN(dir);
        //各種制御の初期化
        ang_v_ctrl.set(pm.slalom_ang_v_P, pm.slalom_ang_v_I, pm.slalom_ang_v_D);
        ang_ctrl.set(pm.slalom_ang_P, pm.slalom_ang_I, pm.slalom_ang_D);
        v_ctrl.set(pm.slalom_v_P, pm.slalom_v_I, pm.slalom_v_D);
        wall_ctrl.set(0.0, 1.0, 0.0);

        //ターゲットの初期化
        trans_t.set(0.0, v, 0.0, DELTA_T);
        rot_t.set(0.0, 0.0, 0.0, DELTA_T);

    };

    Slalom_classic_90deg(float v, float dir, bool search_flag) {
        ParameterManager &pm = ParameterManager::getInstance();

        //変数初期化
        accum_ang = 0.0;
        accum_x = 0.0;
        d_pre = 0.03031937846141211;
        d_fol = 0.03029937846158079;
        if(search_flag == true) {
            d_pre += 0.01;
            d_fol -= 0.01;
        }
        //printfAsync("In slalom classic 90deg\n ");

        rot_dir = SIGN(dir);
        //各種制御の初期化
        ang_v_ctrl.set(pm.slalom_ang_v_P, pm.slalom_ang_v_I, pm.slalom_ang_v_D);
        ang_ctrl.set(pm.slalom_ang_P, pm.slalom_ang_I, pm.slalom_ang_D);
        v_ctrl.set(pm.slalom_v_P, pm.slalom_v_I, pm.slalom_v_D);
        wall_ctrl.set(0.0, 1.0, 0.0);

        //ターゲットの初期化
        trans_t.set(0.0, v, 0.0, DELTA_T);
        rot_t.set(0.0, 0.0, 0.0, DELTA_T);

    };

    bool isEnd() {
        if(accum_x > path_len_90deg_classic + d_pre + d_fol) {
            //printfAsync("Out slalom classic 90deg\n ");
            return true;
        }
        else return false;
    };

    void updateTarget(){
        trans_t.update();
        if(accum_x > d_pre && accum_x < d_pre + path_len_90deg_classic) {
            uint16_t index = uint16_t( (trans_t.x - d_pre) /delta_s_len_90deg_classic);

            if(index <= 0) {
                index = 0;
                rot_t.a = 0.0;
                rot_t.v = 0.0;
                rot_t.x += rot_t.v * DELTA_T;
            }
            else if(index >= kappa_len_90deg_classic - 1) {
                rot_t.a = 0.0;
                rot_t.v = 0.0;
                rot_t.x += rot_t.v * DELTA_T;
            }
            else {
                float rot_t_v_pre = rot_t.v;
                rot_t.v = RAD2DEG(rot_dir * kappa_90deg_classic[index] * trans_t.v);
                rot_t.a = (rot_t.v - rot_t_v_pre)/DELTA_T;
                rot_t.x += rot_t.v * DELTA_T;
            }
        }
        else {
            rot_t.v = 0.0;
            rot_t.a = 0.0;
        }
    }



};


class Slalom_half_90deg : public BaseMoveEvent {

private:
#include "kappa.h"

public:

    float rot_dir;
    float d_pre;
    float d_fol;


    Slalom_half_90deg(float v, float dir) {
        ParameterManager &pm = ParameterManager::getInstance();

        //変数初期化
        accum_ang = 0.0;
        accum_x = 0.0;
        d_pre = d_pre_90deg_half;
        d_fol = d_fol_90deg_half;
        //printfAsync("In slalom classic 90deg\n ");

        rot_dir = SIGN(dir);
        //各種制御の初期化
        ang_v_ctrl.set(pm.slalom_ang_v_P, pm.slalom_ang_v_I, pm.slalom_ang_v_D);
        ang_ctrl.set(pm.slalom_ang_P, pm.slalom_ang_I, pm.slalom_ang_D);
        v_ctrl.set(pm.slalom_v_P, pm.slalom_v_I, pm.slalom_v_D);
        wall_ctrl.set(0.0, 1.0, 0.0);

        //ターゲットの初期化
        trans_t.set(0.0, v, 0.0, DELTA_T);
        rot_t.set(0.0, 0.0, 0.0, DELTA_T);

    };

    Slalom_half_90deg(float v, float dir, bool search_flag) {
        ParameterManager &pm = ParameterManager::getInstance();

        //変数初期化
        accum_ang = 0.0;
        accum_x = 0.0;
        d_pre = d_pre_90deg_half;
        d_fol = d_fol_90deg_half;
        if(search_flag == true) {
            d_pre += 0.005;
            d_fol -= 0.005;
        }
        //printfAsync("In slalom classic 90deg\n ");

        rot_dir = SIGN(dir);
        //各種制御の初期化
        ang_v_ctrl.set(pm.slalom_ang_v_P, pm.slalom_ang_v_I, pm.slalom_ang_v_D);
        ang_ctrl.set(pm.slalom_ang_P, pm.slalom_ang_I, pm.slalom_ang_D);
        v_ctrl.set(pm.slalom_v_P, pm.slalom_v_I, pm.slalom_v_D);
        wall_ctrl.set(0.0, 1.0, 0.0);

        //ターゲットの初期化
        trans_t.set(0.0, v, 0.0, DELTA_T);
        rot_t.set(0.0, 0.0, 0.0, DELTA_T);

    };

    bool isEnd() {
        if(accum_x > path_len_90deg_half + d_pre + d_fol) {
            //printfAsync("Out slalom classic 90deg\n ");
            return true;
        }
        else return false;
    };

    void updateTarget(){
        trans_t.update();
        if(accum_x > d_pre && accum_x < d_pre + path_len_90deg_half) {
            uint16_t index = uint16_t( (trans_t.x - d_pre) /delta_s_len_90deg_half);

            if(index <= 0) {
                index = 0;
                rot_t.a = 0.0;
                rot_t.v = 0.0;
                rot_t.x += rot_t.v * DELTA_T;
            }
            else if(index >= kappa_len_90deg_half - 1) {
                rot_t.a = 0.0;
                rot_t.v = 0.0;
                rot_t.x += rot_t.v * DELTA_T;
            }
            else {
                float rot_t_v_pre = rot_t.v;
                rot_t.v = RAD2DEG(rot_dir * kappa_90deg_half[index] * trans_t.v);
                rot_t.a = (rot_t.v - rot_t_v_pre)/DELTA_T;
                rot_t.x += rot_t.v * DELTA_T;
            }
        }
        else {
            rot_t.v = 0.0;
            rot_t.a = 0.0;
        }
    }
};




class EventList {
public:

    static EventList& getInstance() {
        static EventList instance;
        return instance;
    };
    void update() {
        if(eventList.empty() == false) {
            UMouse &mouse = UMouse::getInstance();
            Vector2f duty = eventList.front()->calcDuty();
            if(eventList.front()->isEnd() == true) {
                delete eventList.front();
                //printfAsync("delete eve \n ");
                eventList.pop();
                duty.x = 0.0;
                duty.y = 0.0;
                mouse.accum_ang = 0.0;
            }
            mouse.setDuty(duty.x, duty.y);

        }
        else {
            UMouse &mouse = UMouse::getInstance();
            ICM20602 &imu = ICM20602::getInstance();
            mouse.accum_ang += 0.001 * imu.omega_f[2];
            fmodf(mouse.accum_ang + 360.0, 360.0);
            if(mouse.accum_ang >180.0) mouse.accum_ang -= 360.0;
        }
    }

    void push(BaseMoveEvent* moveEve) {
        eventList.push(moveEve);
    }

    uint16_t getEventNum() {
        return eventList.size();
    }

    bool empty() {
        return eventList.empty();
    }

    void clear() {
        while (!eventList.empty()) eventList.pop();
    }

private:
    queue<BaseMoveEvent*> eventList;

    EventList() {};
    ~EventList() {};
    EventList(EventList&) {};

};

}
