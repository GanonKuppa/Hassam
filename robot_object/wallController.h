#pragma once



#include "pidController.h"
#include "wallsensor.h"


namespace robot_object{

class WallController : public VelocityTypePidController {

public:
    using VelocityTypePidController::VelocityTypePidController;

    void update(WallSensor& ws) {

        float error;

        int16_t e_fr = 0;
        int16_t e_fl = 0;
        int16_t e_br = 0;
        int16_t e_bl = 0;
        uint8_t valid_error_num = 0;

        if(ws.isRight_for_ctrl() == 1) e_fr = -(ws.right() - ws.center_r() );
        if(ws.isLeft_for_ctrl() == 1) e_fl = +(ws.left() - ws.center_l() );
        //if(ws.isRight_for_ctrl_b() == 1) e_br = +(ws.right_b() - ws.center_r_b());
        //if(ws.isLeft_for_ctrl_b()  == 1) e_bl = -(ws.left_b()  - ws.center_l_b());

        //if(ws.ahead() > 900){ e_fr = 0; e_fl=0;}
        //if(ws.ahead_b() > 900){ e_br = 0; e_bl=0;}

        if(e_fr != 0) valid_error_num ++;
        if(e_fl != 0) valid_error_num ++;
        //if(e_br != 0) valid_error_num ++;
        //if(e_bl != 0) valid_error_num ++;

        if(valid_error_num != 0) error = (float)(e_fr + e_fl + e_br + e_bl) / (float)(valid_error_num);
        else error = 0.0f;

        UMouse &m = UMouse::getInstance();
        e_k0 = -error;

        float delta_u_k = Kp * (e_k0 - e_k1 + T_s / Ki * e_k0 + Kd / T_s * (e_k0 - 2 * e_k1 + e_k2) );
        u_k0 = u_k1 + delta_u_k;

        u_k1 = u_k0;
        e_k2 = e_k1;
        e_k1 = e_k0;

        if(valid_error_num == 0) u_k0 = 0.0;

    };

};



}




