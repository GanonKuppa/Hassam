#pragma once

namespace robot_object{


class FanController {
private:
    FanController();
     ~FanController();

public:
    float front_duty;
    float back_duty;

    void setFrontDuty(float duty);
    void setBackDuty(float duty);
    void setDuty(float duty);

    void setFrontDuty(float duty,bool sb_flag);
    void setBackDuty(float duty, bool sb_flag);

    static FanController& getInstance() {
         static FanController instance;
         return instance;
    }

    void debug();
};


}
