/*
 * mode_RGB.cpp
 *
 *  Created on: 2017/02/25
 *      Author: ryota
 */




#include <mouse.hpp>
#include <stdint.h>
#include <uart.h>
#include "mode_RGB.h"
#include "wallsensor.hpp"
#include "tactsw.h"
#include "timer.h"
#include "fcled.h"
#include "sound.h"
#include "Gamepad.h"
#include "moveEvent.h"

#include "communication.h"

using namespace robot_object;
//using namespace umouse_object;

void mode_RGB(){
    printfAsync("RGB mode\n");
    printfAsync("テストモード\n");
//    MPU9250::getInstance().calibOmegaOffset(200);
//    Icm20608G::getInstance().calibOmegaOffset(200);
    Gamepad &gamepad = Gamepad::getInstance();
    UMouse  &mouse = UMouse::getInstance();
    TactSw &tsw = TactSw::getInstance();
    WallSensor &ws = WallSensor::getInstance();
    EventList &events = EventList::getInstance();

    UMouse &m = UMouse::getInstance();
//    m.switch_back = true;
    while(1){
        waitmsec(1000);
        printfAsync("----");
        printfAsync("%d %d\n",ws.left(), ws.right());
        printfAsync("%d %d\n",ws.center_l(), ws.center_r());

        if(ws.isAhead() == 1){
            SEA();
            waitmsec(3000);
            SEG();
            ws.setWallCenterVal();
  //          events.push(new Trape(0.09, 0.15, 0.0, 0.15, 3.0, true));
//            events.push(new Slalom_half_90deg(0.15, 90.0));
//            events.push(new Trape(0.09, 0.15, 0.15, 0.05, 3.0, true));
//            events.push(new Stop(1000));
        }
/*
        if(ws.isRight() == 1){
            SEA();
            waitmsec(3000);
            events.push(new Trape(0.09, 0.15, 0.0, 0.15, 3.0, true));
            events.push(new Slalom_half_90deg(0.15, -90.0));
            events.push(new Trape(0.09, 0.15, 0.15, 0.05, 3.0, true));
            events.push(new Stop(1000));

        }
*/
        if(ws.isAhead() == 1){
            SEA();
            waitmsec(3000);
            events.push(new Trape(0.09*15, 0.15, 0.0, 0.05, 3.0, true));
        }


        if (tsw.getOntime() > 10 ||  gamepad.B > 100 && gamepad.B < 200 ) {
            mouse.setDuty_R(0.0);
            mouse.setDuty_L(0.0);
            SEB();
            printfAsync("select! \n");
            waitmsec(1000);
            return;
        }

    }

}
