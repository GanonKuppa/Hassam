#pragma once
/*
#include "pidController.h"


namespace robot_object{

class ModeSelector {
private:
    ModeSelector();
    ~ModeSelector();

public:
    float front_duty;
    float back_duty;

    void setFrontDuty(float duty);
    void setBackDuty(float duty);
    void setDuty(float duty);

    void setFrontDuty(float duty,bool sb_flag);
    void setBackDuty(float duty, bool sb_flag);

    static ModeSelector& getInstance() {
         static ModeSelector instance;
         return instance;
    }

    void debug();
};


/*
1のとき
180 0 180 ターゲット0
2のとき
270 0 90  ターゲット0
 90 180 270   ターゲット180
3のとき
300 0 60 ターゲット0
60 120 180 ターゲット120
180 240 0     ターゲット240
*/
/*
class PuseudoDial {
private:
    bool enable_R;
    bool enable_L;
    uint8_t division_num_R;
    uint8_t division_num_L;
    VelocityTypePidController tire_ang_ctrl_R;
    VelocityTypePidController tire_ang_ctrl_L;

    PuseudoDial(){
        division_num_R = 0;
        division_num_L = 0;
        enable_R = false;
        enable_L = false;


    };
    ~PuseudoDial();


public:

    void setEnable(float duty);
    void update(){
        uint8_t mode_change_R = (int)((whOdom.getTireAng_R() / 360.0) * (float)division_num_R);
        uint8_t mode_change_L = (int)((whOdom.getTireAng_R() / 360.0) * (float)division_num_L);

        if(enable == true){
            //target_angle.
            //tire_ang_ctrl_R.get
        }
    };

    void setDivisionNum(uint8_t num){
        division_num = num;
    };

    static PuseudoDial& getInstance() {
         static PuseudoDial instance;
         return instance;
    }

    void debug();
};




}
*/
