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

using namespace robot_object;
using namespace umouse_object;


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
      while(1){
          if(count_start > 3)break;
          if(ws.ahead() > 2000){
              SEA();
              count_start++;
          }
          else count_start = 0;
          waitmsec(500);
      };

      SEB();





    vector<Path> path_vec;
    makeMinStepPath(pm.goal_x, pm.goal_y, m.maze, path_vec);
    //printfAsync("%d %d %d \n",path_vec[0].turn_type, path_vec[0].block_num, path_vec[0].turn_dir);
    //printfAsync("%d %d %d \n",path_vec[1].turn_type, path_vec[1].block_num, path_vec[1].turn_dir);

    printfAsync("--- makeMinStepPath ----\n");
    printPath(path_vec);
    printfAsync("--- jointStraightPath ----\n");
    jointStraightPath(path_vec);

    //printPath(path_vec);
    TurnParameter turn_p(1.0, 0.3, 3.0);

    //fc.setDuty(0.65);
    icm.calibOmegaOffset(400);
    icm.calibAccOffset(400);
    waitmsec(2000);

    playPath(turn_p, path_vec);




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



