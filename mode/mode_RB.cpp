/*
 * mode_N.cpp
 *
 *  Created on: 2017/02/25
 *      Author: ryota
 */




#include <mouse.h>
#include <stdint.h>
#include <uart.h>
#include <wallsensor.h>
#include "mode_R.h"
#include "tactsw.h"
#include "timer.h"
#include "fcled.h"
#include "sound.h"
#include "pwm.h"
#include "gamepad.h"

#include "sound.h"

#include "moveEvent.h"
#include "communication.h"
#include "parameterManager.h"
#include "tactsw.h"

using namespace robot_object;



void mode_RB(){
    printfAsync("RB mode\n");
    printfAsync("ロボトレース\n");
    WallSensor &ws = WallSensor::getInstance();
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



    ICM20602 &icm = ICM20602::getInstance();
    icm.calibOmegaOffset(400);
    icm.calibAccOffset(400);
    waitmsec(2000);


    UMouse &m = UMouse::getInstance();
    EventList &events = EventList::getInstance();
    Gamepad &gamepad = Gamepad::getInstance();
    TactSw &tsw = TactSw::getInstance();
    ParameterManager &pm = ParameterManager::getInstance();
    events.push(new Trace());
    //events.push(new Trape(0.18, pm.tracer_v, pm.tracer_v, 0.0, 2.0,false));

    events.push(new Stop(1000));
    while(1){
        peri::waitusec(1000);
        //Bボタン長押しで抜ける

        if (gamepad.B > 1000 || tsw.getOntime() > 1000 ) {
            SEB();
            printfAsync("select! \n");
            events.clear();
            waitmsec(1000);

            break;
        }
    }

}



