#include <stdint.h>

#include "mode.h"
#include "mode_N.h"
#include "mode_R.h"
#include "mode_G.h"
#include "mode_B.h"
#include "mode_RG.h"
#include "mode_RB.h"
#include "mode_GB.h"
#include "mode_RGB.h"

#include "imu.h"
#include "mouse.h"

#include "myUtil.h"
#include "tactsw.h"
#include "timer.h"
#include "fcled.h"
#include "sound.h"
#include "gamepad.h"
#include "phaseCounting.h"
#include "wallsensor.h"
#include "wheelOdometry.h"
#include "pidController.h"
#include "communication.h"


using std::string;


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
    VelocityTypePidController tire_ang_ctrl_L;
    VelocityTypePidController tire_ang_ctrl_R;
    tire_ang_ctrl_L.set(0.001, 100000.0, 0.0);
    tire_ang_ctrl_R.set(0.001, 100000.0, 0.0);
    float target_ang_L = 0.0;
    float target_ang_R = 0.0;



    wodo.resetTireAng();
    wodo.tire_ang_R = 90.0;

    void (*mode_func[])() = {
        mode_N, mode_R, mode_G, mode_B,
        mode_GB, mode_RB, mode_RG, mode_RGB
    };
    fcled.turn(0, 0, 0);
    printfAsync("############ MD select ################\n");
    printfAsync("Choose the MD .\n");
    printfAsync("++++++++++++++++++++++++++++++++\n");


    while (1) {
        waitmsec(1);
        mode_change = (int)((wodo.getTireAng_L() / 360.0) * (float)mode_num);
        target_ang_L = fmod(360.0 * (float)mode_change / (float)mode_num, 360.0);
        target_ang_R = 90.0;
        tire_ang_ctrl_L.update(target_ang_L, wodo.getTireAng_L());
        tire_ang_ctrl_R.update(target_ang_R, wodo.getTireAng_R());
        float duty_L = -constrain(tire_ang_ctrl_L.getControlVal(), -0.2, 0.2) ;
        float duty_R = -constrain(tire_ang_ctrl_R.getControlVal(), -0.2, 0.2) ;

        m.setDuty_L(duty_L);
        m.setDuty_R(duty_R);

        if(wodo.getTireAng_R() > 260.0 && wodo.getTireAng_R() < 280.0) mode_select = true;

        if(mode_change != mode_change_pre){
            if (mode_change == 0) fcled.turn(0, 0, 0);
            else if (mode_change == 1) fcled.turn(1, 0, 0);
            else if (mode_change == 2) fcled.turn(0, 1, 0);
            else if (mode_change == 3) fcled.turn(0, 0, 1);
            else if (mode_change == 4) fcled.turn(0, 1, 1);
            else if (mode_change == 5) fcled.turn(1, 0, 1);
            else if (mode_change == 6) fcled.turn(1, 1, 0);
            else if (mode_change == 7) fcled.turn(1, 1, 1);
            SEA();
        }
        mode_change_pre = mode_change;

        if (tsw.getOntime() > 10 ){
            while (tsw.getOntime() > 10) {
                waitmsec(1);
                if (tsw.getOntime() > 1000 && tsw.getOntime() < 2000) {
                    //決定時の動作
                    SEB();
                    m.setDuty_L(0.0);
                    m.setDuty_R(0.0);
                    mode_func[mode_change]();
                    waitmsec(1000);
                    return;
                }
            }

            mode_change = (mode_change + 1) % mode_num;
            printfAsync("botton pressed! %d\n", mode_change);

            SEA();
            if (mode_change == 0) fcled.turn(0, 0, 0);
            else if (mode_change == 1) fcled.turn(1, 0, 0);
            else if (mode_change == 2) fcled.turn(0, 1, 0);
            else if (mode_change == 3) fcled.turn(0, 0, 1);
            else if (mode_change == 4) fcled.turn(0, 1, 1);
            else if (mode_change == 5) fcled.turn(1, 0, 1);
            else if (mode_change == 6) fcled.turn(1, 1, 0);
            else if (mode_change == 7) fcled.turn(1, 1, 1);

        }

        if(gamepad.cross_y == 1  ){
            while(gamepad.cross_y==1){
                waitmsec(100);
            }
            wodo.tire_ang_L = fmod(wodo.tire_ang_L + 360.0/mode_num + 360.0, 360.0);
            printfAsync("botton pressed! %d\n", mode_change);
        }

        if(gamepad.cross_y == -1  ){
            while(gamepad.cross_y==-1){
                waitmsec(100);
            }
            wodo.tire_ang_L = fmod(wodo.tire_ang_L - 360.0/mode_num + 360.0, 360.0);
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
            m.setDuty_L(0.0);
            m.setDuty_R(0.0);
            mode_func[mode_change]();
            waitmsec(1000);
            return;
        }


    }
}




int8_t modeSelectLoop(uint8_t mode_num){
    VelocityTypePidController tire_ang_ctrl_L;
    ICM20602 &icm = ICM20602::getInstance();
    WallSensor &ws = WallSensor::getInstance();
    WheelOdometry& whOdom = WheelOdometry::getInstance();
    UMouse& m = UMouse::getInstance();
    FcLed& fcled = FcLed::getInstance();
    uint8_t mode_change = 0;
    uint8_t mode_change_pre = 0;
    uint16_t count_start = 0;
    tire_ang_ctrl_L.set(0.001, 100000.0, 0.0);
    float target_ang = 0.0;

    while (1) {
        mode_change = (int)((whOdom.getTireAng_L() / 360.0) * (float)mode_num);
        target_ang = fmod(360.0 * (float)mode_change / (float)mode_num, 360.0);
        tire_ang_ctrl_L.update(target_ang, whOdom.getTireAng_L());
        float duty = -constrain(tire_ang_ctrl_L.getControlVal(), -0.2, 0.2) ;
        m.setDuty_L(duty);

        if(mode_change != mode_change_pre) SEA();

        if (mode_change == 0) fcled.turn(0, 0, 0);
        else if (mode_change == 1) fcled.turn(1, 0, 0);
        else if (mode_change == 2) fcled.turn(0, 1, 0);
        else if (mode_change == 3) fcled.turn(0, 0, 1);
        else if (mode_change == 4) fcled.turn(0, 1, 1);
        else if (mode_change == 5) fcled.turn(1, 0, 1);
        else if (mode_change == 6) fcled.turn(1, 1, 0);
        else if (mode_change == 7) fcled.turn(1, 1, 1);

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
    m.setDuty_L(0.0);
    return mode_change;
}




