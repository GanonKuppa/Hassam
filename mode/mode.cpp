#include <mouse.hpp>
#include <stdint.h>
#include <uart.h>
#include "mode.h"
#include "mode_N.h"
#include "mode_R.h"
#include "mode_G.h"
#include "mode_B.h"
#include "mode_RG.h"
#include "mode_RB.h"
#include "mode_GB.h"
#include "mode_RGB.h"

#include "imu.hpp"
#include "tactsw.h"
#include "timer.h"
#include "fcled.h"
#include "sound.h"
#include "gamepad.h"
#include "phaseCounting.h"
#include "wallsensor.hpp"
#include "wheelOdometry.hpp"


using std::string;

#include "communication.h"

using namespace robot_object;


void modeSelect() {
    uint8_t mode_change = 0;
    uint8_t mode_change_pre = 0;
    bool mode_select = false;
    uint8_t mode_num = 8;

    ICM20602 &icm = ICM20602::getInstance();
    TactSw& tsw = TactSw::getInstance();
    FcLed& fcled = FcLed::getInstance();
    Gamepad& gamepad = Gamepad::getInstance();
    UMouse& m = UMouse::getInstance();
    WallSensor& ws = WallSensor::getInstance();
    WheelOdometry& wodo =WheelOdometry::getInstance();

    wodo.resetTireAng();


    void (*mode_func[])() = {
        mode_N, mode_B, mode_G, mode_GB,
        mode_R, mode_RB, mode_RG, mode_RGB
    };
    fcled.turn(0, 0, 0);
    printfAsync("############ MD select ################\n");
    printfAsync("Choose the MD .\n");
    printfAsync(" 宴会芸                                        :0 N   \n"); //000 0
    printfAsync(" 自律走行                                    :1 B   \n"); //001 1
    printfAsync(" DEBUG MODE           :2 G   \n"); //010 2
    printfAsync(" 斜め最短走行                             :3 GB  \n"); //011 3
    printfAsync(" 音楽室                                        :4 R   \n"); //100 4
    printfAsync(" 最短走行                                    :5 RB  \n"); //101 5
    printfAsync(" 探索走行                                    :6 RG  \n"); //110 6
    printfAsync(" Circit running       :7 RGB \n"); //111 7
    printfAsync("++++++++++++++++++++++++++++++++\n");


    while (1) {
        waitmsec(1);
        mode_change = (int)((wodo.getTireAng_R() / 360.0) * (float)mode_num);
        if(wodo.getTireAng_L() > 90.0 && wodo.getTireAng_L() < 180.0) mode_select = true;

        if(mode_change != mode_change_pre){
            uint8_t r = (mode_change & 0x04) >> 2;
            uint8_t g = (mode_change & 0x02) >> 1;
            uint8_t b = mode_change & 0x01;
            fcled.turn(r, g, b);
            SEA();
        }
        mode_change_pre = mode_change;

        if (tsw.getOntime() > 10 ){
            while (tsw.getOntime() > 10) {
                waitmsec(1);
                if (tsw.getOntime() > 1000 && tsw.getOntime() < 2000) {
                    //決定時の動作
                    SEB();
                    mode_func[mode_change]();
                    waitmsec(1000);
                    return;
                }
            }

            mode_change = (mode_change + 1) % mode_num;
            printfAsync("botton pressed! %d\n", mode_change);

            SEA();
            uint8_t r = (mode_change & 0x04) >> 2;
            uint8_t g = (mode_change & 0x02) >> 1;
            uint8_t b = mode_change & 0x01;
            fcled.turn(r, g, b);
        }

        if(gamepad.cross_y == 1  ){
            while(gamepad.cross_y==1){
                waitmsec(100);
            }
            wodo.tire_ang_R = fmod(wodo.tire_ang_R + 360.0/mode_num + 360.0, 360.0);
            printfAsync("botton pressed! %d\n", mode_change);
        }

        if(gamepad.cross_y == -1  ){
            while(gamepad.cross_y==-1){
                waitmsec(100);
            }
            wodo.tire_ang_R = fmod(wodo.tire_ang_R - 360.0/mode_num + 360.0, 360.0);
            printfAsync("botton pressed! %d\n", mode_change);

        }

        if(gamepad.X > 50 && gamepad.X < 150 ){
            SEA();
            icm.calibOmegaOffset(400);
            SEF();
            waitmsec(100);
        }

        if(gamepad.Y > 50 && gamepad.Y < 150 ){
            SEA();
            waitmsec(100);
            SEG();
            ws.setWallCenterVal();
            waitmsec(100);
        }

        if(mode_select == true || (gamepad.B > 50 && gamepad.B < 150) ) {
            RAMEN();
            printfAsync("select! \n");
            mode_func[mode_change]();
            waitmsec(1000);
            return;
        }


    }
}

