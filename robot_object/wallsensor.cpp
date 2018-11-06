/*
 * wallsensor.cpp
 *
 *  Created on: 2018/06/09
 *      Author: ryota
 */
#include <mouse.h>
#include <wallsensor.h>

namespace robot_object {

bool WallSensor::isRight() {
    UMouse& m = UMouse::getInstance();
    if (m.switch_back == true) {
        BackWallSensor& back = BackWallSensor::getInstance();
        return back.isRight();
    } else {
        FrontWallSensor& front = FrontWallSensor::getInstance();
        return front.isRight();
    }
}

bool WallSensor::isLeft() {
    UMouse &mouse = UMouse::getInstance();
    UMouse &m = UMouse::getInstance();
    if (m.switch_back == true) {
        BackWallSensor& back = BackWallSensor::getInstance();
        return back.isLeft();
    } else {
        FrontWallSensor& front = FrontWallSensor::getInstance();
        return front.isLeft();
    }
}

bool WallSensor::isAhead() {
    UMouse& m = UMouse::getInstance();
    if (m.switch_back == true) {
        BackWallSensor& back = BackWallSensor::getInstance();
        return back.isAhead();
    } else {
        FrontWallSensor& front = FrontWallSensor::getInstance();
        return front.isAhead();
    }
}

bool WallSensor::isRight_for_ctrl() {
    UMouse& m = UMouse::getInstance();
    if (m.switch_back == true) {
        BackWallSensor& back = BackWallSensor::getInstance();
        return back.isRight_for_ctrl();
    } else {
        FrontWallSensor& front = FrontWallSensor::getInstance();
        return front.isRight_for_ctrl();
    }

}

bool WallSensor::isLeft_for_ctrl() {
    robot_object::UMouse& m = UMouse::getInstance();
    if (m.switch_back == true) {
        BackWallSensor& back = BackWallSensor::getInstance();
        return back.isLeft_for_ctrl();
    } else {
        FrontWallSensor& front = FrontWallSensor::getInstance();
        return front.isLeft_for_ctrl();
    }
}

int16_t WallSensor::right() {
    robot_object::UMouse& m = UMouse::getInstance();
    if (m.switch_back == true) {
        BackWallSensor& back = BackWallSensor::getInstance();
        return back.right.at(0);
    } else {
        FrontWallSensor& front = FrontWallSensor::getInstance();
        return front.right.at(0);
    }

}


int16_t WallSensor::left() {
    robot_object::UMouse& m = UMouse::getInstance();
    if (m.switch_back == true) {
        BackWallSensor& back = BackWallSensor::getInstance();
        return back.left.at(0);
    } else {
        FrontWallSensor& front = FrontWallSensor::getInstance();
        return front.left.at(0);
    }
}
;

int16_t WallSensor::ahead() {
    robot_object::UMouse& m = UMouse::getInstance();
    if (m.switch_back == true) {
        BackWallSensor& back = BackWallSensor::getInstance();
        return back.ahead.at(0);
    } else {
        FrontWallSensor& front = FrontWallSensor::getInstance();
        return front.ahead.at(0);
    }
}


int16_t WallSensor::center_r() {
    robot_object::UMouse& m = UMouse::getInstance();
    ParameterManager &pm = ParameterManager::getInstance();

    if (m.switch_back == true) {
        if(pm.half_flag == 0) return pm.back_wall_center_r;
        else return pm.HF_back_wall_center_r;
    } else {
        if(pm.half_flag == 0) return pm.front_wall_center_r;
        return pm.HF_front_wall_center_r;
    }
}

int16_t WallSensor::center_l() {
    robot_object::UMouse& m = UMouse::getInstance();
    ParameterManager &pm = ParameterManager::getInstance();
    if (m.switch_back == true) {
        if(pm.half_flag == 0) return pm.back_wall_center_l;
        else return pm.HF_back_wall_center_l;
    } else {
        if(pm.half_flag == 0) return pm.front_wall_center_l;
        else return pm.HF_front_wall_center_l;
    }
}



bool WallSensor::isRight_for_ctrl_b() {
    UMouse& m = UMouse::getInstance();
    if (m.switch_back == false) {
        BackWallSensor& back = BackWallSensor::getInstance();
        return back.isLeft_for_ctrl();
    } else {
        FrontWallSensor& front = FrontWallSensor::getInstance();
        return front.isLeft_for_ctrl();
    }

}

bool WallSensor::isLeft_for_ctrl_b() {
    robot_object::UMouse& m = UMouse::getInstance();
    if (m.switch_back == false) {
        BackWallSensor& back = BackWallSensor::getInstance();
        return back.isRight_for_ctrl();
    } else {
        FrontWallSensor& front = FrontWallSensor::getInstance();
        return front.isRight_for_ctrl();
    }
}

int16_t WallSensor::right_b() {
    robot_object::UMouse& m = UMouse::getInstance();
    if (m.switch_back == false) {
        BackWallSensor& back = BackWallSensor::getInstance();
        return back.left.at(0);
    } else {
        FrontWallSensor& front = FrontWallSensor::getInstance();
        return front.left.at(0);
    }

}


int16_t WallSensor::left_b() {
    robot_object::UMouse& m = UMouse::getInstance();
    if (m.switch_back == false) {
        BackWallSensor& back = BackWallSensor::getInstance();
        return back.right.at(0);
    } else {
        FrontWallSensor& front = FrontWallSensor::getInstance();
        return front.right.at(0);
    }
}
;

int16_t WallSensor::ahead_b() {
    robot_object::UMouse& m = UMouse::getInstance();
    if (m.switch_back == false) {
        BackWallSensor& back = BackWallSensor::getInstance();
        return back.ahead.at(0);
    } else {
        FrontWallSensor& front = FrontWallSensor::getInstance();
        return front.ahead.at(0);
    }
}

int16_t WallSensor::center_r_b() {
    robot_object::UMouse& m = UMouse::getInstance();
    ParameterManager &pm = ParameterManager::getInstance();

    if (m.switch_back == false) {
        return pm.back_wall_center_l;
    } else {
        return pm.front_wall_center_l;
    }
}

int16_t WallSensor::center_l_b() {
    robot_object::UMouse& m = UMouse::getInstance();
    ParameterManager &pm = ParameterManager::getInstance();
    if (m.switch_back == false) {
        return pm.back_wall_center_r;
    } else {
        return pm.front_wall_center_r;
    }
}





}

