/*
 * fanController.cpp
 *
 *  Created on: 2018/06/16
 *      Author: ryota
 */

#include <fanController.h>
#include <myUtil.h>
#include "iodefine.h"
#include "pwm.h"
#include "timer.h"

namespace peri = peripheral_RX71M;
using peri:: waitmsec;

namespace robot_object {

FanController::FanController() {
    front_duty = 0.0;
    back_duty = 0.0;
}

FanController::~FanController() {
}

void FanController::setFrontDuty(float duty) {
    front_duty = constrain(duty, -1.0, 1.0);
    if (duty > 0) {
        PORT4.PODR.BIT.B1 = 1; //MOTOR_IN1_2 ファン マイコンない側
        PORT4.PODR.BIT.B2 = 0; //MOTOR_IN2_2
        peri::setDutyMTU3(duty);
    }
    if (duty < 0) {
        PORT4.PODR.BIT.B1 = 0; //MOTOR_IN1_2 ファン マイコンない側
        PORT4.PODR.BIT.B2 = 1; //MOTOR_IN2_2
        peri::setDutyMTU3(ABS(duty));
    }
    if (duty == 0.0) {
        PORT4.PODR.BIT.B1 = 0; //MOTOR_IN1_2 ファン マイコンない側
        PORT4.PODR.BIT.B2 = 0; //MOTOR_IN2_2
        peri::setDutyMTU3(0.0);
    }

};


void FanController::setBackDuty(float duty) {
    back_duty = constrain(duty, -1.0, 1.0);
    if (duty > 0) {
        PORTA.PODR.BIT.B1 = 0; //MOTOR_IN1_4 ファン マイコン側
        PORTA.PODR.BIT.B3 = 1; //MOTOR_IN2_4
        peri::setDutyMTU7(duty);
    }
    if (duty < 0) {
        PORTA.PODR.BIT.B1 = 1; //MOTOR_IN1_4 ファン マイコン側
        PORTA.PODR.BIT.B3 = 0; //MOTOR_IN2_4
        peri::setDutyMTU7(ABS(duty));
    }
    if (duty == 0.0) {
        PORTA.PODR.BIT.B1 = 0; //MOTOR_IN1_4 ファン マイコン側
        PORTA.PODR.BIT.B3 = 0; //MOTOR_IN2_4
        peri::setDutyMTU7(0.0);

    }

};


void FanController::setDuty(float duty) {
    setFrontDuty(duty);
    setBackDuty(duty);
};

void FanController::setFrontDuty(float duty,bool sb_flag){
    if(sb_flag == true) setBackDuty(duty);
    else setFrontDuty(duty);
};

void FanController::setBackDuty(float duty, bool sb_flag){
    if(sb_flag == true) setFrontDuty(duty);
    else setBackDuty(duty);
};

void FanController::debug(){
    FanController::setDuty(0.1);
    waitmsec(4000);
    FanController::setDuty(0.0);
    waitmsec(2000);
    FanController::setDuty(0.2);
    waitmsec(4000);
    FanController::setDuty(0.0);
    waitmsec(2000);
    FanController::setDuty(0.3);
    waitmsec(4000);
    FanController::setDuty(0.0);
    waitmsec(2000);
    FanController::setDuty(0.4);
    waitmsec(4000);
    FanController::setDuty(0.0);
    waitmsec(2000);
    FanController::setDuty(-0.1);
    waitmsec(4000);
    FanController::setDuty(-0.0);
    waitmsec(2000);
    FanController::setDuty(-0.2);
    waitmsec(4000);
    FanController::setDuty(-0.0);
    waitmsec(2000);
    FanController::setDuty(-0.3);
    waitmsec(4000);
    FanController::setDuty(0.0);
    waitmsec(2000);
    FanController::setDuty(-0.4);
    waitmsec(4000);
    FanController::setDuty(0.0);
    waitmsec(2000);

}



}
