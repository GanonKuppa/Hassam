/*
 * mode_B.cpp
 *
 *  Created on: 2017/02/25
 *      Author: ryota
 */

#include <mouse.hpp>
#include <stdint.h>
#include <uart.h>
#include "mode_B.h"
#include "tactsw.h"
#include "timer.h"
#include "fcled.h"
#include "sound.h"
#include "gamepad.h"
#include "imu.hpp"
#include "fanController.hpp"

//Eigen
//#include <Core>
//#include <Geometry>

#include "moveEvent.h"
#include "communication.h"

using namespace robot_object;

void mode_B(){
    printfAsync("B mode\n");

//    MPU9250::getInstance().calibOmegaOffset(200);
//    MPU9250::getInstance().calibAccOffset(200);
    ICM20602 &icm = ICM20602::getInstance();
    Gamepad &gamepad = Gamepad::getInstance();
    UMouse  &mouse   = UMouse::getInstance();
    FanController &fc = FanController::getInstance();


    while(1){
        waitmsec(1);

        EventList &events = EventList::getInstance();

        if(gamepad.X > 0 && gamepad.X < 50 ){
            SEA();
            events.push(new Trape(0.18*8,0.1,0.0,0.0,3.0, true));
            events.push(new Stop(1000));
            waitmsec(100);
        }

        if(gamepad.Y > 0 && gamepad.Y < 50 ){
            SEA();
            events.push(new Trape(0.18*8,0.2,0.0,0.0,3.0, true));
            events.push(new Stop(1000));
            waitmsec(100);
        }


        if(gamepad.LB > 0 && gamepad.LB < 50){
            SEA();
            events.push(new Trape(0.18*8,0.3,0.0,0.0,3.0, true));
            events.push(new Stop(1000));
            waitmsec(100);
        }
        if(gamepad.RB > 0 && gamepad.RB < 50){
            SEA();
            events.push(new Trape(0.18*8,0.4,0.0,0.0,3.0, true));
            events.push(new Stop(1000));
            waitmsec(100);
        }

        if(gamepad.START > 0 && gamepad.START < 50){
            SEA();
            events.push(new Trape(0.18*8,0.5,0.0,0.0,3.0, true));
            events.push(new Stop(1000));
            waitmsec(100);
        }

        if(gamepad.BACK > 50 && gamepad.BACK < 50){
            SEA();
            events.push(new Trape(0.18*8,1.0,0.0,0.0,3.0, true));
            events.push(new Stop(1000));
            waitmsec(100);
        }


        if(gamepad.cross_x == 1){
            events.push(new PivotTurn(-90.0));
            events.push(new Stop(50));
            waitmsec(300);
        }
        if(gamepad.cross_x == -1){
            events.push(new PivotTurn(90.0));
            events.push(new Stop(50));
            waitmsec(300);
        }

        if(gamepad.cross_y == 1){
            events.push(new Trape(0.09,0.4,0.0,0.0,2.0, false));
            events.push(new Stop(50));
            waitmsec(300);
        }
        if(gamepad.cross_y == -1){
            events.push(new PivotTurn(180.0));
            events.push(new Stop(50));
            waitmsec(300);
        }



        if (gamepad.B > 0 && gamepad.B < 50 ) {
            SEB();
            printfAsync("select! \n");
            waitmsec(1000);
            return;
        }

    }

};
