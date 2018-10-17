/*
 * communication.cpp
 *
 *  Created on: 2017/08/13
 *      Author: ryota
 */


#include <mouse.hpp>
#include "communication.h"

#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>

#include "uart.h"
#include "timeInterrupt.h"
#include "timer.h"
#include "ad.h"
#include "phaseCounting.h"
#include "pwm.h"

#include "sound.h"
#include "imu.hpp"
#include "tactsw.h"
#include "fcled.h"
#include "wallsensor.hpp"
#include "gamepad.h"
#include "WheelOdometry.hpp"
#include "ICM20602.hpp"

#include "maze.h"
#include "parameterManager.h"

#include "myUtil.hpp"

#include <deque>
#include <queue>


using std::deque;
using std::queue;

namespace peri = peripheral_RX71M;



static void set2ByteVal(uint8_t *buf, uint16_t index, float val,float prop);
static void set2ByteVal(uint8_t *buf, uint16_t index, uint16_t val);
static void set2ByteVal(uint8_t *buf, uint16_t index, int16_t val);
static void set2ByteVal(uint8_t *buf, uint16_t index, uint32_t val);
static void set2ByteVal(uint8_t *buf, uint16_t index, int32_t val);
static void set4ByteVal(uint8_t *buf, uint16_t index, uint32_t val);

namespace robot_object{

    static const uint16_t PERIODIC_MSG_LEN = 400;
    static const uint16_t CMD_SIZE  = 16;
    static queue<uint8_t> printfBuff;
    static uint8_t periodicMsg[PERIODIC_MSG_LEN];

    static void packDataMaze(uint8_t part_num, uint8_t *buf);
    static void packData(uint8_t *buf);

    /***********同期printf関数******************/
    int printfSync(const char *fmt, ...) {
        static char buffer[1000];
        int len;

        va_list ap;
        va_start(ap, fmt);

        len = vsprintf(buffer, fmt, ap);
        peri::putnbyteSCI1(buffer, len);
        va_end(ap);
        return len;
    }


    /***********非同期printf関数******************/
    int printfAsync(const char *fmt, ...) {
        static char buffer[1000];
        int len;

        va_list ap;
        va_start(ap, fmt);

        len = vsprintf(buffer, fmt, ap);

        for (int c = 0; c < len; c++) {
            printfBuff.push(buffer[c]);
        }

        va_end(ap);
        return len;
    }


    void fetchCommand()
    {
        static bool first_recieve_flag = false;
        int16_t last_cmd_index = -1;
        for(int i=0;i<peri::recieveBuffCount-4;i++){
            if(peri::recieveBuffCount-i > CMD_SIZE &&
                peri::recieveBuff[i+0]==99 &&
                peri::recieveBuff[i+1]==109 &&
                peri::recieveBuff[i+2]==100
            ){

                if(first_recieve_flag == false){
                    SEE();
                    first_recieve_flag = true;
                }

                if(peri::recieveBuff[i+3] == 254 && peri::recieveBuff[i+4] == 253){
                    Gamepad &gamepad = Gamepad::getInstance();
                    gamepad.updateCommand(&peri::recieveBuff[i]);
                }

                if(peri::recieveBuff[i+3] == 251){
                    ParameterManager &pm = ParameterManager::getInstance();

                    pm.writeCommand(&peri::recieveBuff[i]);
                }

                last_cmd_index = i;
                break;
            }
        }

        if (last_cmd_index != -1){
            for(int ind = last_cmd_index + CMD_SIZE;
                    ind < peri::recieveBuffCount;
                    ind++){
                peri::recieveBuff[ind -(last_cmd_index + CMD_SIZE)] = peri::recieveBuff[ind];
            }
            peri::recieveBuffCount = peri::recieveBuffCount - (last_cmd_index + CMD_SIZE);
        }
    }

    /***********periodicMsgを送る******************/
    void sendPeriodicMsg() {
        packData(periodicMsg);
        peri::putnbyteSCIFA9(periodicMsg, PERIODIC_MSG_LEN);
    }

    //迷路の壁情報を送る
    //32 x 32の迷路データを4つに分割
    void packDataMaze(uint8_t part_num, uint8_t *buf) {
        buf[0] = part_num;
        UMouse &mouse = UMouse::getInstance();
        uint8_t ind = 1;

        if (part_num == 0) {
            for (int i = 0; i < 16; i++) {
                buf[ind + 0 + i * 4] = ((mouse.maze.walls_vertical[i] & 0x000000ff)
                        >> 0);
                buf[ind + 1 + i * 4] = ((mouse.maze.walls_vertical[i] & 0x0000ff00)
                        >> 8);
                buf[ind + 2 + i * 4] = ((mouse.maze.walls_vertical[i] & 0x00ff0000)
                        >> 16);
                buf[ind + 3 + i * 4] = ((mouse.maze.walls_vertical[i] & 0xff000000)
                        >> 24);
            }
        }

        if (part_num == 1) {
            for (int i = 0; i < 15; i++) {
                buf[ind + 0 + i * 4] = ((mouse.maze.walls_vertical[i + 16]
                        & 0x000000ff) >> 0);
                buf[ind + 1 + i * 4] = ((mouse.maze.walls_vertical[i + 16]
                        & 0x0000ff00) >> 8);
                buf[ind + 2 + i * 4] = ((mouse.maze.walls_vertical[i + 16]
                        & 0x00ff0000) >> 16);
                buf[ind + 3 + i * 4] = ((mouse.maze.walls_vertical[i + 16]
                        & 0xff000000) >> 24);
            }
        }

        if (part_num == 2) {
            for (int i = 0; i < 16; i++) {
                buf[ind + 0 + i * 4] =
                        ((mouse.maze.walls_horizontal[i] & 0x000000ff) >> 0);
                buf[ind + 1 + i * 4] =
                        ((mouse.maze.walls_horizontal[i] & 0x0000ff00) >> 8);
                buf[ind + 2 + i * 4] =
                        ((mouse.maze.walls_horizontal[i] & 0x00ff0000) >> 16);
                buf[ind + 3 + i * 4] =
                        ((mouse.maze.walls_horizontal[i] & 0xff000000) >> 24);
            }
        }

        if (part_num == 3) {
            for (int i = 0; i < 15; i++) {
                buf[ind + 0 + i * 4] = ((mouse.maze.walls_horizontal[i + 16]
                        & 0x000000ff) >> 0);
                buf[ind + 1 + i * 4] = ((mouse.maze.walls_horizontal[i + 16]
                        & 0x0000ff00) >> 8);
                buf[ind + 2 + i * 4] = ((mouse.maze.walls_horizontal[i + 16]
                        & 0x00ff0000) >> 16);
                buf[ind + 3 + i * 4] = ((mouse.maze.walls_horizontal[i + 16]
                        & 0xff000000) >> 24);
            }
        }
    }

    //パラメータマネージャーに登録された変数を配列に格納
    //52byteを占有 型情報  パート番号2byte + (1byte + 変数値 4byte) *10 = 52byte
    void packDataParamMng(uint8_t part_num, uint8_t *buf) {
        buf[0] = part_num;
        buf[1] = 255;
        ParameterManager &pm = ParameterManager::getInstance();
        for(uint8_t i=0;i<10;i++){
            if(pm.typeMap.find(10*part_num + i) != pm.typeMap.end()){
                uint16_t val_num = 10*part_num +i;
                Type_e type = pm.typeMap[val_num];
                buf[2+i*5] = (uint8_t)type;
                if(type == Type_e::FLOAT) *reinterpret_cast<float*>(&buf[2+i*5+1]) = *reinterpret_cast<float*>(pm.adrMap[val_num]);
                if(type == Type_e::UINT8) *reinterpret_cast<uint8_t*>(&buf[2+i*5+1]) = *reinterpret_cast<uint8_t*>(pm.adrMap[val_num]);
                if(type == Type_e::UINT16) *reinterpret_cast<uint16_t*>(&buf[2+i*5+1]) = *reinterpret_cast<uint16_t*>(pm.adrMap[val_num]);
                if(type == Type_e::UINT32) *reinterpret_cast<uint32_t*>(&buf[2+i*5+1]) = *reinterpret_cast<uint32_t*>(pm.adrMap[val_num]);
                if(type == Type_e::INT8) *reinterpret_cast<int8_t*>(&buf[2+i*5+1]) = *reinterpret_cast<int8_t*>(pm.adrMap[val_num]);
                if(type == Type_e::INT16) *reinterpret_cast<int16_t*>(&buf[2+i*5+1]) = *reinterpret_cast<int16_t*>(pm.adrMap[val_num]);
                if(type == Type_e::INT32) *reinterpret_cast<int32_t*>(&buf[2+i*5+1]) = *reinterpret_cast<int32_t*>(pm.adrMap[val_num]);

            }
            else{
                buf[2+i*5] = 255;
            }
        }
    }


    void packData(uint8_t *buf) {
        uint8_t printfDataNum = 0;
        const uint8_t printfFieldNum = 20;

        TactSw& tsw = TactSw::getInstance();
        UMouse &m = UMouse::getInstance();
        FrontWallSensor &frontWallSen = FrontWallSensor::getInstance();
        BackWallSensor &backWallSen = BackWallSensor::getInstance();
        WheelOdometry &whOdom = WheelOdometry::getInstance();
        ICM20602 &icm = ICM20602::getInstance();

        //header
        buf[0] = 0xff;
        buf[1] = 0xff;
        buf[2] = 0x48;
        buf[3] = 0x45;
        buf[4] = 0x41;
        buf[5] = 0x44;
        uint32_t elapsedTime = getElapsedMsec();
        set4ByteVal(buf, 8, elapsedTime);

        set2ByteVal(buf, 12, peri::getAD_AN102());
        set2ByteVal(buf, 14, peri::getAD_AN101());
        set2ByteVal(buf, 16, peri::getAD_AN109());
        set2ByteVal(buf, 18, peri::getAD_AN113());
        set2ByteVal(buf, 20, peri::getAD_AN110());
        set2ByteVal(buf, 22, peri::getAD_AN004());
        set2ByteVal(buf, 24, peri::getAD_AN005());
        set2ByteVal(buf, 26, peri::getAD_AN105());

        set2ByteVal(buf, 28, peri::getDutyMTU4(),32767);
        set2ByteVal(buf, 30, peri::getDutyMTU3(),32767);
        set2ByteVal(buf, 32, peri::getDutyMTU0(),32767);
        set2ByteVal(buf, 34, peri::getDutyMTU7(),32767);

        set2ByteVal(buf, 36, peri::getTimeuCountIntCMT0());
        set2ByteVal(buf, 38, peri::getTimeuCountIntCMT1());
        set2ByteVal(buf, 40, TPU0.TCNT);

        set2ByteVal(buf, 42, peri::getCountMTU1());
        set2ByteVal(buf, 44, peri::getCountMTU2());

        set2ByteVal(buf, 46, tsw.getOntime());

        set2ByteVal(buf, 48, icm.omega_raw[0]);
        set2ByteVal(buf, 50, icm.omega_raw[1]);
        set2ByteVal(buf, 52, icm.omega_raw[2]);
        set2ByteVal(buf, 54, icm.acc_raw[0]);
        set2ByteVal(buf, 56, icm.acc_raw[1]);
        set2ByteVal(buf, 58, icm.acc_raw[2]);

        set2ByteVal(buf, 60, m.start.x);
        set2ByteVal(buf, 62, m.start.y);
        set2ByteVal(buf, 64, m.goal.x);
        set2ByteVal(buf, 66, m.goal.y);
        set2ByteVal(buf, 68, whOdom.ab_pos_x, 3000);
        set2ByteVal(buf, 70, whOdom.ab_pos_y, 3000);
        set2ByteVal(buf, 72, (int16_t)m.direction);

        set2ByteVal(buf, 74, (int16_t)frontWallSen.right.at(0));
        set2ByteVal(buf, 76, (int16_t)frontWallSen.ahead.at(0));
        set2ByteVal(buf, 78, (int16_t)frontWallSen.left.at(0));

        set2ByteVal(buf, 80, (int16_t)backWallSen.right.at(0));
        set2ByteVal(buf, 82, (int16_t)backWallSen.ahead.at(0));
        set2ByteVal(buf, 84, (int16_t)backWallSen.left.at(0));

        set2ByteVal(buf, 86, m.t_ang_a, 3.0);
        set2ByteVal(buf, 88, m.t_ang_v, 20.0);
        set2ByteVal(buf, 90, m.t_ang, 20.0);
        set2ByteVal(buf, 92, m.accum_ang, 100.0);
        set2ByteVal(buf, 94, icm.omega_f[2], 15.0);
        set2ByteVal(buf, 96, whOdom.ang_v, 15.0);

        set2ByteVal(buf, 98, m.t_a, 1000.0);
        set2ByteVal(buf, 100, m.t_v, 3000.0);
        set2ByteVal(buf, 102, m.t_x, 3000.0);
        set2ByteVal(buf, 104, m.accum_x, 3000.0);

        set2ByteVal(buf, 106, icm.acc_f[1], 1500.0); //縦G
        set2ByteVal(buf, 108, icm.acc_f[0], 1500.0); //横G
        set2ByteVal(buf, 110, whOdom.v, 3000.0 );
        set2ByteVal(buf, 112, m.acc_v, 3000.0 );

        set2ByteVal(buf, 114, whOdom.ab_ang, 80.0 );
        /*
        set2ByteVal(buf, 122, m.v_enc, 3000.0 );
        set2ByteVal(buf, 124, m.wall_P, 20.0);
        set2ByteVal(buf, 126, m.wall_I, 20.0);
        set2ByteVal(buf, 128, m.wall_D, 20.0);
        set2ByteVal(buf, 130, m.v_P, 20.0);
        set2ByteVal(buf, 132, m.v_I, 20.0);
        set2ByteVal(buf, 134, m.v_D, 20.0);
        set2ByteVal(buf, 136, m.ang_P, 20.0);
        set2ByteVal(buf, 138, m.ang_I, 20.0);
        set2ByteVal(buf, 140, m.ang_D, 20.0);
        set2ByteVal(buf, 142, m.ang_v_P, 20.0);
        set2ByteVal(buf, 144, m.ang_v_I, 20.0);
        set2ByteVal(buf, 146, m.ang_v_D, 20.0);
        set2ByteVal(buf, 148, m.v_FF, 20.0);
        set2ByteVal(buf, 150, m.ang_FF, 20.0);

        set2ByteVal(buf, 152, m.ab_ang, 100.0);
*/
        //迷路データ
        static uint8_t count = 0;
        packDataMaze(count, &buf[160]);
        count++;
        if (count == 4) count = 0;

        //パラメータマネージャのデータ
        static uint8_t count_paramMng = 0;
        packDataParamMng(count_paramMng, &buf[250]);
        count_paramMng++;
        if(count_paramMng == 20) count_paramMng = 0;

        //printf Data
        uint16_t start_byte = PERIODIC_MSG_LEN - printfFieldNum;
        uint16_t end_byte = PERIODIC_MSG_LEN;
        for (int i = start_byte; i < end_byte; i++) {
            if (printfBuff.empty() == false) {
                buf[i] = printfBuff.front();
                printfBuff.pop();
                printfDataNum++;
            } else {
                buf[i] = 0;
            }
        }
        //printf Data Num
        buf[7] = printfDataNum;

        //check sum
        uint8_t sum = 0;
        for (int i = 7; i < PERIODIC_MSG_LEN; i++)
            sum += buf[i];
        buf[6] = sum;
    }

    uint8_t* getPointerOfPeriodicMsg() {
        return periodicMsg;
    }


}

inline void set2ByteVal(uint8_t *buf, uint16_t index, float val,float prop){
    int16_t int_val = (int16_t)(val * prop);
    buf[index]   = (0x0000ff00 & int_val) >> 8;
    buf[index+1] = (0x000000ff & int_val)   ;
}

inline void set2ByteVal(uint8_t *buf, uint16_t index, uint16_t val){
    buf[index]   = (0x0000ff00 & val) >> 8;
    buf[index+1] = (0x000000ff & val)   ;
}

inline void set2ByteVal(uint8_t *buf, uint16_t index, int16_t val){
    buf[index]   = (0x0000ff00 & val) >> 8;
    buf[index+1] = (0x000000ff & val)   ;
}

inline void set2ByteVal(uint8_t *buf, uint16_t index, uint32_t val){
    buf[index]   = (0x0000ff00 & val) >> 8;
    buf[index+1] = (0x000000ff & val)   ;
}

inline void set2ByteVal(uint8_t *buf, uint16_t index, int32_t val){
    buf[index]   = (0x0000ff00 & val) >> 8;
    buf[index+1] = (0x000000ff & val)   ;
}

inline void set4ByteVal(uint8_t *buf, uint16_t index, uint32_t val){
    buf[index] = ((0xff000000 & val) >> 24);
    buf[index+1] = ((0x00ff0000 & val) >> 16);
    buf[index+2] = ((0x0000ff00 & val) >> 8);
    buf[index+3] = ((0x000000ff & val));
}





