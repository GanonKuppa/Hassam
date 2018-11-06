/*
 * mode_N.cpp
 *
 *  Created on: 2017/02/25
 *      Author: ryota
 */




#include <mouse.hpp>
#include <stdint.h>
#include <uart.h>
#include "mode_R.h"
#include "tactsw.h"
#include "timer.h"
#include "fcled.h"
#include "sound.h"
#include "pwm.h"
#include "gamepad.h"

#include "sound.h"

#include "ICM20602.hpp"
#include "moveEvent.h"
#include "communication.h"
#include "parameterManager.h"
#include "wallsensor.hpp"
#include <pathCalculation.hpp>
#include <vector>
#include "tactsw.h"
#include "fanController.hpp"
#include "wheelOdometry.hpp"

using namespace robot_object;
using namespace umouse_object;


static bool mode_select;
static uint8_t mode_change;
static uint8_t mode_change_pre;
static uint8_t mode_num;
static uint16_t count_start;

static void init(){
    WheelOdometry& whOdom = WheelOdometry::getInstance();
    FcLed& fcled = FcLed::getInstance();

    //mode関連初期化
    mode_select = false;
    mode_change = 0;
    mode_change_pre = 0;
    mode_num = 8;
    whOdom.resetTireAng();
    count_start = 0;

}

static int8_t modeSelectLoop(){
    ICM20602 &icm = ICM20602::getInstance();
    WallSensor &ws = WallSensor::getInstance();
    WheelOdometry& whOdom = WheelOdometry::getInstance();
    FcLed& fcled = FcLed::getInstance();

    while (1) {
        mode_change = (int)((whOdom.getTireAng_R() / 360.0) * (float)mode_num);
        if(mode_change != mode_change_pre) SEA();

        if (mode_change == 0) fcled.turn(1, 0, 0);      //パラメータ0
        else if (mode_change == 1) fcled.turn(0, 1, 0); //パラメータ1
        else if (mode_change == 2) fcled.turn(0, 0, 1); //パラメータ2
        else if (mode_change == 3) fcled.turn(0, 1, 1); //パラメータ3
        else if (mode_change == 4) fcled.turn(1, 0, 1); //パラメータ4
        else if (mode_change == 5) fcled.turn(1, 1, 0); //パラメータ5
        else if (mode_change == 6) fcled.turn(1, 1, 1); //パラメータ6
        else if (mode_change == 7) fcled.turn(0, 0, 0); //モードセレクトへ帰る

        mode_change_pre = mode_change;

        if (count_start > 1500) break;
        if (ws.ahead() > 2500) {
            count_start++;
            if(count_start % 500 == 0) SEA();
        }
        else count_start = 0;
        waitmsec(1);
    };
    SEB();
    waitmsec(1000);
    return 0;
}





void mode_R(){
    printfAsync("R mode\n");
    printfAsync("最短\n");

    UMouse &m = UMouse::getInstance();
    EventList &events = EventList::getInstance();
    Gamepad &gamepad = Gamepad::getInstance();
    ParameterManager &pm = ParameterManager::getInstance();
    FanController &fc = FanController::getInstance();
    WallSensor &ws = WallSensor::getInstance();
    uint16_t count_start = 0;
    ICM20602 &icm = ICM20602::getInstance();
    TactSw &tsw = TactSw::getInstance();


    TurnParameter turn_p;

    init();
    modeSelectLoop();
    if(mode_change == 0) turn_p.set(0.6, 0.5, 4.0);
    else if(mode_change == 1) turn_p.set(1.0, 0.5 ,4.0);
    else if(mode_change == 2) turn_p.set(1.5, 0.5 ,4.0);
    else if(mode_change == 3) turn_p.set(0.6, 0.3 ,4.0);
    else if(mode_change == 4) turn_p.set(1.0, 0.3 ,4.0);
    else if(mode_change == 5) turn_p.set(1.2, 0.3 ,4.0);
    else if(mode_change == 6) turn_p.set(1.5, 0.3 ,4.0);
    else if(mode_change == 7) return;


    vector<Path> path_vec;
    makeMinStepPath(pm.goal_x, pm.goal_y, m.maze, path_vec);

    printfAsync("--- makeMinStepPath ----\n");
    printPath(path_vec);
    printfAsync("--- jointStraightPath ----\n");
    jointStraightPath(path_vec);

    icm.calibOmegaOffset(400);
    icm.calibAccOffset(400);

    if(pm.half_flag == 1){
        if(mode_change == 0 || mode_change == 1 || mode_change == 2) HF_playPathPivot(turn_p, path_vec);
        else if(mode_change == 3 || mode_change == 4 || mode_change == 5 || mode_change == 6) HF_playPath(turn_p, path_vec);
    }
    else{
        if(mode_change == 0 || mode_change == 1 || mode_change == 2) playPathPivot(turn_p, path_vec);
        else if(mode_change == 3 || mode_change == 4 || mode_change == 5 || mode_change == 6) playPath(turn_p, path_vec);
    }




    while(1){
        peri::waitusec(100);
        //fc.setDuty(0.65);
        //Bボタン長押しで抜ける
        if (tsw.getOntime() > 10 ) {
            SEB();
            printfAsync("select! \n");
            fc.setDuty(0.0);
            events.clear();
            m.setDuty_L(0.0);
            m.setDuty_R(0.0);
            waitmsec(1000);
            break;
        }
        //printfAsync("while\n");

        //if(isEmptyBgmList() == 1)addBgmList(owen);
     }
    fc.setDuty(0.0);
}



