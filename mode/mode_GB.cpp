#include <mouse.h>
#include <myUtil.h>
#include <pathCalculation.h>
#include <stdint.h>
#include <uart.h>
#include <wallsensor.h>
#include "mode_GB.h"
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


static bool mode_select;
static uint8_t mode_change;
static uint8_t mode_change_pre;
static uint8_t mode_num;

static float v; //直進の最高速度
static float a; //直進の加速度
static float v_max;
static float v_min;
static float bl_len;
static float read_wall_offset;
static float wall_2_section_center_len;
static int8_t rot_times;
static uint16_t count_start;

static void init(){
    UMouse &m = UMouse::getInstance();
    EventList &events = EventList::getInstance();
    Gamepad &gamepad = Gamepad::getInstance();
    ParameterManager &pm = ParameterManager::getInstance();

    WallSensor &ws = WallSensor::getInstance();
    FrontWallSensor& fws = FrontWallSensor::getInstance();
    BackWallSensor& bws = BackWallSensor::getInstance();
    WheelOdometry& whOdom = WheelOdometry::getInstance();
    FcLed& fcled = FcLed::getInstance();
    TactSw &tsw = TactSw::getInstance();

    if(pm.half_flag == 0){
        bl_len = 0.18;
        read_wall_offset = 0.01;
        wall_2_section_center_len = 0.045;
    }
    else{
        bl_len = 0.09;
        read_wall_offset = 0.005;
        wall_2_section_center_len = 0.005;
    }

    //mode関連初期化
    mode_select = false;
    mode_change = 0;
    mode_change_pre = 0;
    mode_num = 4;
    whOdom.resetTireAng();
    uint16_t count_start = 0;
    //探索パラメータ初期化
    v = pm.v_search_run; //探索速度
    a = 3.0; //直進の加速度
    v_max = 1.0;
    v_min = 0.1;
    rot_times = 0;


    //座標関連初期化
    m.goal.set(pm.goal_x, pm.goal_y);
    m.start.set(0, 0);
    m.coor.set(0, 0);
    whOdom.ab_pos_x = bl_len/2.0;
    whOdom.ab_pos_y = bl_len/2.0;
    whOdom.ab_ang = 90.0;
    m.direction = N;

}

static int8_t modeSelectLoop(){
    ICM20602 &icm = ICM20602::getInstance();
    WallSensor &ws = WallSensor::getInstance();
    WheelOdometry& whOdom = WheelOdometry::getInstance();
    FcLed& fcled = FcLed::getInstance();

    while (1) {
        mode_change = (int)((whOdom.getTireAng_R() / 360.0) * (float)mode_num);
        if(mode_change != mode_change_pre) SEA();

        if (mode_change == 0) fcled.turn(1, 0, 0);  //行き帰りスラ探
        else if (mode_change == 1) fcled.turn(0, 1, 0); //行きスラ探
        else if (mode_change == 2) fcled.turn(0, 0, 1); //行き帰り超信知探索
        else if (mode_change == 3) fcled.turn(1, 1, 1); //行き超信知探索

        mode_change_pre = mode_change;

        if (count_start > 1500) break;
        if (ws.ahead() > 1500) {
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

static int8_t startSectionRun(){
    UMouse &m = UMouse::getInstance();
    EventList &events = EventList::getInstance();

    events.push(new Stop(500));
    m.maze.updateWall(m.coor.x, m.coor.y, m.direction, WallSensor::getInstance());
    events.push(new Trape(bl_len/2.0 - read_wall_offset, v, 0.0, v, a, true));
    m.coor.y++;
    return 0;
}

static int8_t searchRunLoop(uint8_t destination_x, uint8_t destination_y){
    UMouse &m = UMouse::getInstance();
    EventList &events = EventList::getInstance();
    WallSensor& ws = WallSensor::getInstance();
    FrontWallSensor& fws = FrontWallSensor::getInstance();
    BackWallSensor& bws = BackWallSensor::getInstance();
    Gamepad &gamepad = Gamepad::getInstance();
    ParameterManager &pm = ParameterManager::getInstance();
    TactSw &tsw = TactSw::getInstance();

    while (1) {
       peri::waitusec(100);

       //if(isEmptyBgmList() == 1)addBgmList(meiji);

       //Bボタン長押しで抜ける
       if (gamepad.B > 1000 || tsw.getOntime() > 10){
//               || ((fws.ahead.at(0) > 1000) && (fws.left.at(0) > 1000) && (fws.right.at(0) > 1000)
//                       && (bws.ahead.at(0) > 1000) && (bws.left.at(0) > 1000) && (bws.right.at(0) > 1000))) {
           SEB();
           printfAsync("select! \n");
           waitmsec(1000);
           events.clear();
           m.setDuty_R(0.0);
           m.setDuty_L(0.0);

           return -1;
       }
       //printfAsync("while\n");

       if (events.empty() == true) {
           //---壁を更新
           SEA();
           printfAsync("----------\n");
           printfAsync("pos :(%d, %d) %d\n", m.coor.x, m.coor.y, m.direction);
           printfAsync("SB  :%d\n", m.switch_back);
           printfAsync("sen :%d %d %d\n", ws.left(), ws.ahead(), ws.right());

           m.maze.updateWall(m.coor.x, m.coor.y, m.direction, ws);
           //---ゴールに着いたら探索終了
           if (m.coor == m.goal) {
               events.push(new Trape(bl_len/2.0 + read_wall_offset, v, v, v_min, a, true));
               events.push(new Stop(1000));
               famima();
               return 0;
           }

           m.maze.makeSearchMap(destination_x, destination_y);
           direction_e dest_dir = m.maze.getMinDirection(m.coor.x, m.coor.y, m.direction);
           rot_times = m.maze.calcRotTimes(dest_dir, m.direction);

           printfAsync("%d MinDirection\n", rot_times);

           if (rot_times == 0 || rot_times == 4) {
               events.push(new Trape(bl_len / 4.0, v, v, v, a, true));
               while (events.empty() == false) peri::waitusec(10);
               SEA();
               int8_t wall_info = m.maze.updateWall(m.coor.x, m.coor.y, m.direction, ws);
               printfAsync("再チェック: %d\n", wall_info);
               if (wall_info == -1) {
                   //現在区画の壁情報を修正する
                   m.maze.writeWall(m.coor.x, m.coor.y, m.direction, ws);
                   SEC();

                   //行先を再算出
                   m.maze.makeSearchMap(destination_x, destination_y);
                   dest_dir = m.maze.getMinDirection(m.coor.x, m.coor.y, m.direction);
                   rot_times = m.maze.calcRotTimes(dest_dir, m.direction);
                   printfAsync("%d 新たなMinDirection\n", rot_times);

                   events.push(new Trape(read_wall_offset + bl_len / 4.0, v, v, v_min, a, true));
                   while (events.empty() == false)
                       peri::waitusec(10);

                   if (rot_times == 4) {
                       if (ws.isAhead() == true) {
                           //尻当て
                           events.push(new Trape(bl_len/2.0, 0.2, 0.2, 0.0, a, true));
                           events.push(new Stop(50));
                           events.push(new SwitchBack());
                           //events.push(new PivotTurn(45.0f * rot_times));
                           events.push(new Trape(wall_2_section_center_len + bl_len/2.0 - read_wall_offset, v, 0.0, v, a, true));
                           events.push(new Stop(50));
                       }
                       else{
                           events.push(new Stop(50));
                           events.push(new SwitchBack());
                           //events.push(new PivotTurn(45.0f * rot_times));
                           events.push(new Trape(bl_len/2.0 - read_wall_offset, v, 0.0, v, a, true));
                       }
                   }
                   else if(rot_times == 0 || ABS(rot_times) == 2) {
                       events.push(new PivotTurn(45.0f * rot_times));
                       events.push(new Trape(bl_len/2.0 - read_wall_offset, v, v_min, v, a, true));
                   }

               }
               else {
                   if (rot_times == 0) {
                       printfAsync("問題なし まっすぐ\n");
                       events.push(new Trape(bl_len * 3.0 / 4.0, v, v, v, a, true));
                   }
                   else if (rot_times == 4) {
                       //printfAsync("問題なし すいっちばっく\n");
                       if (ws.isAhead() == true) {
                          //尻当て
                          events.push(new Trape(bl_len * 3.0/4.0, 0.2, 0.2, 0.0, a, true));
                          events.push(new Stop(50));
                          events.push(new SwitchBack());
                          //events.push(new PivotTurn(45.0f * rot_times));
                          events.push(new Trape(wall_2_section_center_len + bl_len/2.0 - read_wall_offset, v, 0.0, v, a, true));
                          events.push(new Stop(50));
                       }
                       else{
                          events.push(new Stop(50));
                          events.push(new SwitchBack());
                          //events.push(new PivotTurn(45.0f * rot_times));
                          events.push(new Trape(bl_len/2.0 - read_wall_offset, v, 0.0, v, a, true));
                       }


                   }
               }
           }
           else if (ABS(rot_times) == 2) {
               printfAsync("問題なし 曲がる\n");
               if(mode_change == 0 || mode_change == 1){
                   if(pm.half_flag == 0) events.push(new Slalom_classic_90deg(v, 45.0f * rot_times, true));
                   else events.push(new Slalom_half_90deg(v, 45.0f * rot_times, true));
               }else{
                   events.push(new Trape(read_wall_offset + bl_len/2.0, v, v, v_min, a, true));
                   events.push(new Stop(50));
                   events.push(new PivotTurn(45.0f * rot_times));
                   events.push(new Stop(50));
                   events.push(new Trape(bl_len/2.0 - read_wall_offset, v, 0.0, v, a, true));
               }
           }

           m.direction = m.maze.getMinDirection(m.coor.x, m.coor.y, m.direction);
           if (m.direction == E) m.coor.x++;
           else if (m.direction == N) m.coor.y++;
           else if (m.direction == W) m.coor.x--;
           else if (m.direction == S) m.coor.y--;
       }
   }

}

int8_t goalSectionRun(){
    UMouse &m = UMouse::getInstance();
    EventList &events = EventList::getInstance();

    m.maze.makeSearchMap(m.start.x, m.start.y);
    direction_e dest_dir = m.maze.getMinDirection(m.coor.x, m.coor.y, m.direction);
    rot_times = m.maze.calcRotTimes(dest_dir, m.direction);


    if (rot_times == 0) {
        //何もしない
    }
    else if (rot_times == 4) {
        events.push(new SwitchBack());
        //events.push(new PivotTurn(45.0f * rot_times));
    }
    else {
        events.push(new PivotTurn(45.0f * rot_times));
    }
    events.push(new Stop(50));
    events.push(new Trape(bl_len/2.0 - read_wall_offset, v, 0.0, v, a, true));

    m.direction = m.maze.getMinDirection(m.coor.x, m.coor.y, m.direction);
    if (m.direction == E) m.coor.x++;
    else if (m.direction == N) m.coor.y++;
    else if (m.direction == W) m.coor.x--;
    else if (m.direction == S) m.coor.y--;
    return 0;
}


void mode_GB() {
    UMouse &m = UMouse::getInstance();
    ICM20602 &icm = ICM20602::getInstance();

    printfAsync("G mode\n");
    printfAsync("探索モード\n");

    init();
    modeSelectLoop();

    icm.calibOmegaOffset(400);
    icm.calibAccOffset(400);
    //addBgmList(wily);
    if(startSectionRun() == -1) return;
    if(searchRunLoop(m.goal.x, m.goal.y) == -1) return;
    if (mode_change == 1 || mode_change == 3) return;
    if(goalSectionRun() == -1) return;
    if(searchRunLoop(m.start.x, m.start.y) == -1) return;

}


