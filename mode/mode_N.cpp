/*
 * mode_N.cpp
 *
 *  Created on: 2017/02/25
 *      Author: ryota
 */




#include <mouse.h>
#include <myUtil.h>
#include <pathCalculation.h>
#include <stdint.h>
#include <uart.h>
#include <wallsensor.h>
#include "mode_N.h"
#include "tactsw.h"
#include "timer.h"
#include "fcled.h"
#include "sound.h"
#include "pwm.h"
#include "gamepad.h"

#include "sound.h"
#include "ICM20602.h"
#include "moveEvent.h"
#include "communication.h"
#include "parameterManager.h"
#include <vector>

using std::vector;

using namespace robot_object;
using namespace umouse_object;


void mode_N(){
    printfAsync("N mode\n");
    printfAsync("ハーフ探索\n");
    //MPU9250::getInstance().calibOmegaOffset(400);
    //MPU9250::getInstance().calibAccOffset(400);


    UMouse &m = UMouse::getInstance();
    EventList &events = EventList::getInstance();
    Gamepad &gamepad = Gamepad::getInstance();
    ParameterManager &pm = ParameterManager::getInstance();
    ICM20602 &icm = ICM20602::getInstance();
    WallSensor &ws = WallSensor::getInstance();

    m.goal.set(pm.goal_x, pm.goal_y);
    m.start.set(0,0);
    m.coor.set(0,0);
    //m.ab_position.x = 0.09;
    //m.ab_position.y = 0.09;
    //m.ab_ang = 90.0;
    m.direction = N;

    float v = 0.2;        //直進の最高速度
    float v_0 = 0.0;
    float a = 3.0;        //直進の加速度
    float v_max = 1.0;
    int8_t rot_times;
    uint16_t count_start = 0;
    while(1){
        if(count_start > 20)break;
        if(ws.ahead() > 2000){
            count_start++;
            SEA();
        }
        else count_start = 0;
        waitmsec(100);
    };
    SEB();
    waitmsec(1000);
    icm.calibOmegaOffset(400);
    icm.calibAccOffset(400);
    //addBgmList(wily);
    events.push(new Stop(500));
    m.maze.updateWall(m.coor.x, m.coor.y, m.direction, WallSensor::getInstance());
    events.push(new Trape(0.04, v, 0.0, v, a,true));
    m.coor.y++;

    while(1){
        peri::waitusec(100);
        //Bボタン長押しで抜ける
        //if(isEmptyBgmList() == 1)addBgmList(meiji);


        if (gamepad.B > 1000 ) {
            SEB();
            printfAsync("select! \n");
            waitmsec(1000);
            break;
        }
        //printfAsync("while\n");

        if(events.empty() == true){
            //---壁を更新
            printfAsync("----------\n");
            printfAsync("%d %d %d\n", m.coor.x, m.coor.y,m.direction);
            m.maze.updateWall(m.coor.x, m.coor.y, m.direction, WallSensor::getInstance());
            //---ゴールに着いたら探索終了
            if(m.coor == m.goal){
                events.push(new Trape(0.05, v, v, 0.0, a, true));
                events.push(new Stop(200));
                events.push(new Stop(200));
                famima();
                break;
            }
            printfAsync("updateWall\n");
            m.maze.makeSearchMap(m.goal.x, m.goal.y);
            //m.maze.watchPotentialMap();
            printfAsync("makeSerchMap\n");
            rot_times = m.maze.getMinDirection(m.coor.x, m.coor.y, m.direction) - m.direction;
            if (rot_times == 6) rot_times = -2;
            else if(rot_times == -6) rot_times = 2;
            printfAsync("%d getMinDirection\n", rot_times);


            if(rot_times == 0){
                events.push(new Trape(0.09, v, v, v, a, true));
            }
            else if(rot_times == 4 || rot_times == -4){
                events.push(new Trape(0.05, v, v, 0.1, a, true));
                //events.push(new Trape(0.09, 0.1, 0.1, 0.0, a, true));
                events.push(new Stop(50));
                events.push(new PivotTurn(45.0f * rot_times));
                //events.push(new SwitchBack());
                events.push(new Stop(50));
                //events.push(new Trape(0.08+0.043, v, 0.0, v, a, true));
                events.push(new Trape(0.04, v, 0.0, v, a, true));
            }
            //else if (rot_times == 2 || rot_times == -2){
            //    events.push(new Slalom_classic_90deg(v, SIGN(rot_times), true));
            //}
            else{
               events.push(new Trape(0.05, v, v, 0.0, a, true));
               events.push(new Stop(50));
               events.push(new PivotTurn(45.0f * rot_times));
               events.push(new Stop(50));
               events.push(new Trape(0.04, v, 0.0, v, a, true));
           }

            m.direction = m.maze.getMinDirection(m.coor.x, m.coor.y, m.direction);
            if(m.direction == E)m.coor.x ++;
            else if(m.direction == N)m.coor.y++;
            else if(m.direction == W)m.coor.x--;
            else if(m.direction == S)m.coor.y--;
        }
    }


};



