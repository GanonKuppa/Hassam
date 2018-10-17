#pragma once

#include "iodefine.h"
#include "stdint.h"
#include "parameterManager.h"
#include "ad.h"
#include <deque>
#include "myUtil.hpp"
#include "communication.h"
#include "timer.h"


using std::deque;
namespace peri = peripheral_RX71M;

namespace robot_object {

class FrontWallSensor {

private:
    const uint8_t BUFF_SIZE = 5;
    bool enable;

    FrontWallSensor() {
        enable = true;

        ahead_on = 0;
        left_on = 0;
        right_on = 0;

        ahead_off = 0;
        left_off = 0;
        right_off = 0;
        for (uint8_t i = 0; i < BUFF_SIZE; i++) {
            ahead.push_front(0);
            right.push_front(0);
            left.push_front(0);
        }

    }

    FrontWallSensor(FrontWallSensor&) {
        enable = true;
        ahead_on = 0;
        left_on = 0;
        right_on = 0;

        ahead_off = 0;
        left_off = 0;
        right_off = 0;
        for (uint8_t i = 0; i < BUFF_SIZE; i++) {
            ahead.push_front(0);
            right.push_front(0);
            left.push_front(0);
        }
    }

    ~FrontWallSensor() {
        enable = false;
    }

public:

    int16_t ahead_on;
    int16_t left_on;
    int16_t right_on;

    int16_t ahead_off;
    int16_t left_off;
    int16_t right_off;

    deque<int16_t> ahead;
    deque<int16_t> left;
    deque<int16_t> right;

    void turnOffAllLed() {
        PORTE.PODR.BIT.B2 = 0; //SLED_OUT1
        PORTD.PODR.BIT.B6 = 0; //SLED_OUT2
        PORTD.PODR.BIT.B7 = 0; //SLED_OUT3
    }

    void turnOnAllLed() {
        if (enable == true) {
            PORTE.PODR.BIT.B2 = 1; //SLED_OUT1
            PORTD.PODR.BIT.B6 = 1; //SLED_OUT2
            PORTD.PODR.BIT.B7 = 1; //SLED_OUT3
        } else {
            turnOffAllLed();
        }
    }

    void updateAllOffVal() {
        right_off = peri::startAD_AN113(); //AD_SLED2
        left_off = peri::startAD_AN109(); //AD_SLED1
        ahead_off = peri::startAD_AN110(); //AD_SLED3
    }

    void updateAllOnVal() {
        right_on = peri::startAD_AN113();
        left_on = peri::startAD_AN109();
        ahead_on = peri::startAD_AN110();
    }

    void turnOnAheadLed() {
        PORTE.PODR.BIT.B2 = 1; //前
    }

    void turnOnLeftLed() {
        PORTD.PODR.BIT.B7 = 1; //左
    }

    void turnOnRightLed() {
        PORTD.PODR.BIT.B6 = 1; //右
    }

    void updateAheadOnVal() {
        ahead_on = peri::startAD_AN110(); //前センサ
    }

    void updateLeftOnVal() {
        left_on = peri::startAD_AN109(); //左センサ
    }

    void updateRightOnVal() {
        right_on = peri::startAD_AN113(); //右センサ
    }

    void modulateVal() {
        int16_t ah_mod = ahead_on - ahead_off;
        int16_t l_mod = left_on - left_off;
        int16_t r_mod = right_on - right_off;

        ahead.push_front(ah_mod);
        left.push_front(l_mod);
        right.push_front(r_mod);

        ahead.pop_back();
        left.pop_back();
        right.pop_back();
    }

    static FrontWallSensor& getInstance() {
        static FrontWallSensor instance;
        return instance;
    }

    void setEnable(bool en) {
        enable = en;
        if (enable == false)
            turnOffAllLed();
    }

    bool isRight() {
        ParameterManager &pm = ParameterManager::getInstance();
        int16_t threshold_right;
        if(pm.half_flag == 0)threshold_right = pm.front_wall_threshold_right;
        else threshold_right = pm.HF_front_wall_threshold_right;


        if (right.at(0) > threshold_right)
            return true;
        else
            return false;
    }
    ;

    bool isLeft() {
        ParameterManager &pm = ParameterManager::getInstance();
        int16_t threshold_left;
        if(pm.half_flag == 0) threshold_left = pm.front_wall_threshold_left;
        else threshold_left = pm.HF_front_wall_threshold_left;

        if (left.at(0) > threshold_left)
            return true;
        else
            return false;
    }

    bool isAhead() {
        ParameterManager &pm = ParameterManager::getInstance();
        int16_t threshold_ahead;
        if(pm.half_flag == 0) threshold_ahead = pm.front_wall_threshold_ahead;
        else threshold_ahead = pm.HF_front_wall_threshold_ahead;

        if (ahead.at(0) > threshold_ahead)
            return true;
        else
            return false;
    }


    bool isRight_for_ctrl() {
        ParameterManager &pm = ParameterManager::getInstance();
        int16_t d_RS = right.at(0) - right.at(1);
        int16_t threshold_right;
        if(pm.half_flag == 0) threshold_right = pm.front_wall_threshold_right;
        else threshold_right = pm.HF_front_wall_threshold_right;

        int16_t threshold_ctrl = threshold_right;
        ////////右壁////////////////////////
        if (ABS(d_RS) > 10)
            threshold_ctrl += 450;


        if (right.at(0) > threshold_ctrl && right.at(1) > threshold_ctrl &&
            right.at(2) > threshold_ctrl && right.at(3) > threshold_ctrl &&
            right.at(4) > threshold_ctrl) {
            return true;
        } else {
            return false;
        }
    }

    bool isLeft_for_ctrl() {
        ParameterManager &pm = ParameterManager::getInstance();
        int16_t d_LS = left.at(0) - left.at(1);
        int16_t threshold_left;

        if(pm.half_flag == 0)threshold_left = pm.front_wall_threshold_left;
        else threshold_left = pm.HF_front_wall_threshold_left;
        int16_t threshold_ctrl = threshold_left;
        ////////右壁////////////////////////
        if (ABS(d_LS) > 10)
            threshold_ctrl += 450;

        if (left.at(0) > threshold_ctrl && left.at(1) > threshold_ctrl &&
            left.at(2) > threshold_ctrl && left.at(3) > threshold_ctrl &&
            left.at(4) > threshold_ctrl) {
            return true;
        } else {
            return false;
        }
    }

};

class BackWallSensor {

private:
    const uint8_t BUFF_SIZE = 5;
    bool enable;

    BackWallSensor() {
        enable = true;
        ahead_on = 0;
        left_on = 0;
        right_on = 0;

        ahead_off = 0;
        left_off = 0;
        right_off = 0;
        for (uint8_t i = 0; i < BUFF_SIZE; i++) {
            ahead.push_front(0);
            right.push_front(0);
            left.push_front(0);
        }

    }

    BackWallSensor(BackWallSensor&) {
        enable = true;
        ahead_on = 0;
        left_on = 0;
        right_on = 0;

        ahead_off = 0;
        left_off = 0;
        right_off = 0;
        for (uint8_t i = 0; i < BUFF_SIZE; i++) {
            ahead.push_front(0);
            right.push_front(0);
            left.push_front(0);
        }

    }

    ~BackWallSensor() {
        enable = false;
    }

public:

    int16_t ahead_on;
    int16_t left_on;
    int16_t right_on;

    int16_t ahead_off;
    int16_t left_off;
    int16_t right_off;

    deque<int16_t> ahead;
    deque<int16_t> left;
    deque<int16_t> right;

    void turnOffAllLed() {
        PORT1.PODR.BIT.B2 = 0; //SLED_OUT4 配線ミスを繋ぎ直し
        PORTC.PODR.BIT.B2 = 0; //SLED_OUT5
        PORTC.PODR.BIT.B3 = 0; //SLED_OUT6
    }

    void turnOnAllLed() {
        if (enable == true) {
            PORT1.PODR.BIT.B2 = 1; //SLED_OUT4 配線ミスを繋ぎ直し
            PORTC.PODR.BIT.B2 = 1; //SLED_OUT5
            PORTC.PODR.BIT.B3 = 1; //SLED_OUT6
        } else {
            turnOffAllLed();
        }
    }

    void updateAllOffVal() {
        right_off = peri::startAD_AN105(); //AD_SLED4
        left_off = peri::startAD_AN004();  //AD_SLED5
        ahead_off = peri::startAD_AN005(); //AD_SLED6
    }

    void updateAllOnVal() {
        right_on = peri::startAD_AN105(); //AD_SLED4
        left_on = peri::startAD_AN004();  //AD_SLED5
        ahead_on = peri::startAD_AN005(); //AD_SLED6
    }

    void turnOnAheadLed() {
        PORTC.PODR.BIT.B2 = 1;
    }

    void turnOnLeftLed() {
        PORTC.PODR.BIT.B3 = 1;
    }

    void turnOnRightLed() {
        PORT1.PODR.BIT.B2 = 1;
    }

    void updateAheadOnVal() {
        ahead_on = peri::startAD_AN005();
    }

    void updateLeftOnVal() {
        left_on = peri::startAD_AN004();
    }

    void updateRightOnVal() {
        right_on = peri::startAD_AN105();
    }

    void modulateVal() {
        int16_t ah_mod = ahead_on - ahead_off;
        int16_t l_mod = left_on - left_off;
        int16_t r_mod = right_on - right_off;

        ahead.push_front(ah_mod);
        left.push_front(l_mod);
        right.push_front(r_mod);

        ahead.pop_back();
        left.pop_back();
        right.pop_back();
    }

    static BackWallSensor& getInstance() {
        static BackWallSensor instance;
        return instance;
    }

    void setEnable(bool en) {
        enable = en;
        if (enable == false)
            turnOffAllLed();
    }

    bool isRight() {
        ParameterManager &pm = ParameterManager::getInstance();
        int16_t threshold_right;
        if(pm.half_flag == 0)threshold_right = pm.back_wall_threshold_right;
        else threshold_right = pm.HF_back_wall_threshold_right;
        if (right.at(0) > threshold_right)
            return true;
        else
            return false;
    }

    bool isLeft() {
        ParameterManager &pm = ParameterManager::getInstance();
        int16_t threshold_left;
        if(pm.half_flag == 0)threshold_left = pm.back_wall_threshold_left;
        else threshold_left = pm.HF_back_wall_threshold_left;
        if (left.at(0) > threshold_left)
            return true;
        else
            return false;
    }

    bool isAhead() {
        ParameterManager &pm = ParameterManager::getInstance();
        int16_t threshold_ahead;
        if(pm.half_flag == 0)threshold_ahead = pm.back_wall_threshold_ahead;
        else threshold_ahead = pm.HF_back_wall_threshold_ahead;
        if (ahead.at(0) > threshold_ahead)
            return true;
        else
            return false;
    }

    bool isRight_for_ctrl() {
        ParameterManager &pm = ParameterManager::getInstance();
        int16_t d_RS = right.at(0) - right.at(1);
        int16_t threshold_right;
        if(pm.half_flag == 0)threshold_right = pm.back_wall_threshold_right;
        else threshold_right = pm.back_wall_threshold_right;


        int16_t threshold_ctrl = threshold_right;
        ////////右壁////////////////////////
        if (ABS(d_RS) > 10)
            threshold_ctrl += 650;

        if (right.at(0) > threshold_ctrl && right.at(1) > threshold_ctrl &&
            right.at(2) > threshold_ctrl && right.at(3) > threshold_ctrl &&
            right.at(4) > threshold_ctrl) {
            return true;
        } else {
            return false;
        }
    }

    bool isLeft_for_ctrl() {
        ParameterManager &pm = ParameterManager::getInstance();
        int16_t d_LS = left.at(0) - left.at(1);

        int16_t threshold_left;
        if(pm.half_flag == 0)threshold_left = pm.back_wall_threshold_left;
        else threshold_left = pm.back_wall_threshold_left;

        int16_t threshold_ctrl = threshold_left;
        ////////右壁////////////////////////
        if (ABS(d_LS) > 10)
            threshold_ctrl += 650;

        if (left.at(0) > threshold_ctrl && left.at(1) > threshold_ctrl &&
            left.at(2) > threshold_ctrl && left.at(3) > threshold_ctrl &&
            left.at(4) > threshold_ctrl) {
            return true;
        } else {
            return false;
        }
    }

};

class WallSensor {

private:
    const uint8_t BUFF_SIZE = 5;
    bool enable;

    WallSensor() {
        enable = true;
    }

    WallSensor(WallSensor&) {
        enable = true;
    }

    ~WallSensor() {
        enable = false;
    }

public:

    void turnOffAllLed() {
        robot_object::FrontWallSensor& front =
                robot_object::FrontWallSensor::getInstance();
        BackWallSensor& back = BackWallSensor::getInstance();
        front.turnOffAllLed();
        back.turnOffAllLed();
    }

    void turnOnAllLed() {
        FrontWallSensor& front = FrontWallSensor::getInstance();
        BackWallSensor& back = BackWallSensor::getInstance();

        if (enable == true) {
            front.turnOnAllLed();
            back.turnOnAllLed();
        } else {
            front.turnOffAllLed();
            back.turnOffAllLed();
        }
    }

    void updateAllOffVal() {
        FrontWallSensor& front = FrontWallSensor::getInstance();
        BackWallSensor& back = BackWallSensor::getInstance();
        front.updateAllOffVal();
        back.updateAllOffVal();
    }

    void updateAllOnVal() {
        FrontWallSensor& front = FrontWallSensor::getInstance();
        BackWallSensor& back = BackWallSensor::getInstance();
        front.updateAllOnVal();
        back.updateAllOnVal();

    }

    void modulateVal() {
        FrontWallSensor& front = FrontWallSensor::getInstance();
        BackWallSensor& back = BackWallSensor::getInstance();
        front.modulateVal();
        back.modulateVal();
    }

    static WallSensor& getInstance() {
        static WallSensor instance;
        return instance;
    }

    void setEnable(bool en) {
        FrontWallSensor& front = FrontWallSensor::getInstance();
        BackWallSensor& back = BackWallSensor::getInstance();

        enable = en;
        if (enable == false) {
            front.turnOffAllLed();
            back.turnOffAllLed();
        }
    }

    bool isRight();
    bool isLeft();
    bool isAhead();
    bool isRight_for_ctrl();
    bool isLeft_for_ctrl();
    int16_t right();
    int16_t left();
    int16_t ahead();
    int16_t center_r();
    int16_t center_l();


    bool isRight_for_ctrl_b();
    bool isLeft_for_ctrl_b();
    int16_t right_b();
    int16_t left_b();
    int16_t ahead_b();
    int16_t center_r_b();
    int16_t center_l_b();


    void update() {
        FrontWallSensor& front = FrontWallSensor::getInstance();
        BackWallSensor& back = BackWallSensor::getInstance();

        if(enable == true){
            updateAllOffVal();
            // front側On時の値取得
            front.turnOnAllLed();
            peri::waitusec_sub(20);
            front.updateAllOnVal();
            turnOffAllLed();

            // back側On時の値取得
            back.turnOnAllLed();
            peri::waitusec_sub(20);
            back.updateAllOnVal();
            turnOffAllLed();
            modulateVal();
        }
        else{
            turnOffAllLed();
        }


    }

    void setWallCenterVal(){
        FrontWallSensor& front = FrontWallSensor::getInstance();
        BackWallSensor& back = BackWallSensor::getInstance();
        ParameterManager &pm = ParameterManager::getInstance();

        if(pm.half_flag == 0){
            pm.front_wall_center_r = front.right.at(0);
            pm.front_wall_center_l = front.left.at(0);
            pm.back_wall_center_r = back.right.at(0);
            pm.back_wall_center_l = back.left.at(0);
            pm.write<uint16_t>(103, pm.front_wall_center_r);
            pm.write<uint16_t>(104, pm.front_wall_center_l);
            pm.write<uint16_t>(105, pm.back_wall_center_r);
            pm.write<uint16_t>(106, pm.back_wall_center_l);
            printfAsync("----------\n");
            printfAsync("front l r: %d, %d\n",pm.front_wall_center_l, pm.front_wall_center_r);
            printfAsync("back  r l: %d, %d\n",pm.back_wall_center_r, pm.front_wall_center_l);
            printfAsync("----------\n");
        }
        else{
            pm.HF_front_wall_center_r = front.right.at(0);
            pm.HF_front_wall_center_l = front.left.at(0);
            pm.HF_back_wall_center_r = back.right.at(0);
            pm.HF_back_wall_center_l = back.left.at(0);
            pm.write<uint16_t>(115, pm.HF_front_wall_center_r);
            pm.write<uint16_t>(116, pm.HF_front_wall_center_l);
            pm.write<uint16_t>(117, pm.HF_back_wall_center_r);
            pm.write<uint16_t>(118, pm.HF_back_wall_center_l);
            printfAsync("----------\n");
            printfAsync("front l r: %d, %d\n",pm.HF_front_wall_center_l, pm.HF_front_wall_center_r);
            printfAsync("back  r l: %d, %d\n",pm.HF_back_wall_center_r, pm.HF_front_wall_center_l);
            printfAsync("----------\n");


        }

    }


};



}

