#pragma once

#include "stdint.h"
#include "maze.h"
#include "myUtil.hpp"
#include <map> // pair
#include <vector>
#include "communication.h"
#include "moveEvent.h"
#include "timer.h"

using std::vector;
using std::pair;

using namespace robot_object;
namespace umouse_object {

enum turn_type_e {
    STRAIGHT = 0,
    TURN_90,
    TURN_L_90,
    TURN_180,
    TURN_S2D_45,
    TURN_S2D_135,
    TURN_D_90,
    TURN_D2S_45,
    TURN_D2S_135,
    D_STRAIGHT
};

enum turn_dir_e {
    CW = -1, NO_TURN = 0, CCW = 1
};

class Path {
public:
    turn_type_e turn_type;
    uint8_t block_num;
    turn_dir_e turn_dir;

    Path() {
        turn_type = STRAIGHT;
        block_num = 0;
        turn_dir = NO_TURN;
    }

    Path(turn_type_e turn_type_, uint8_t block_num_, turn_dir_e turn_dir_) {
        turn_type = turn_type_;
        turn_dir = turn_dir_;
        block_num = block_num_;
    }

    Path(const Path &obj) {
        this->block_num = obj.block_num;
        this->turn_dir = obj.turn_dir;
        this->turn_type = obj.turn_type;
    }

    ~Path() {

    }

    void print() {
        printfAsync("turn_type %d, block_num %d, turn_dir %d\n", turn_type, block_num, turn_dir);
    }

};

class TurnParameter {
public:
    float v_straight;
    float v_d_straight;
    float v_turn_90;
    float v_turn_l_90;
    float v_turn_180;
    float v_turn_d_90;
    float v_turn_45;
    float v_turn_135;
    float a_straight;
    float a_d_straight;

    TurnParameter() {
        v_straight = 0.0;
        v_d_straight = 0.0;
        v_turn_90 = 0.0;
        v_turn_l_90 = 0.0;
        v_turn_180 = 0.0;
        v_turn_d_90 = 0.0;
        v_turn_45 = 0.0;
        v_turn_135 = 0.0;
        a_straight = 0.0;
        a_d_straight = 0.0;
    }

    TurnParameter(float v, float a) {
        v_straight = v;
        v_d_straight = v;
        v_turn_90 = v;
        v_turn_l_90 = v;
        v_turn_180 = v;
        v_turn_d_90 = v;
        v_turn_45 = v;
        v_turn_135 = v;
        a_straight = a;
        a_d_straight = a;
    }

    TurnParameter(float v, float v_turn, float a) {
        v_straight = v;
        v_d_straight = v;
        v_turn_90 = v_turn;
        v_turn_l_90 = v_turn;
        v_turn_180 = v_turn;
        v_turn_d_90 = v_turn;
        v_turn_45 = v_turn;
        v_turn_135 = v_turn;
        a_straight = a;
        a_d_straight = a;
    }

    void set(float v_straight_, float v_d_straight_, float v_turn_90_, float v_turn_l_90_, float v_turn_180_,
            float v_turn_d_90_, float v_turn_45_, float v_turn_135_, float a_straight_, float a_d_straight_) {
        v_straight = v_straight_;
        v_d_straight = v_d_straight_;
        v_turn_90 = v_turn_90_;
        v_turn_l_90 = v_turn_l_90_;
        v_turn_180 = v_turn_180_;
        v_turn_d_90 = v_turn_d_90_;
        v_turn_45 = v_turn_45_;
        v_turn_135 = v_turn_135_;
        a_straight = a_straight_;
        a_d_straight = a_d_straight_;
    }

    void set(float v, float a) {
        v_straight = v;
        v_d_straight = v;
        v_turn_90 = v;
        v_turn_l_90 = v;
        v_turn_180 = v;
        v_turn_d_90 = v;
        v_turn_45 = v;
        v_turn_135 = v;
        a_straight = a;
        a_d_straight = a;
    }

    void set(float v, float v_turn, float a) {
        v_straight = v;
        v_d_straight = v;
        v_turn_90 = v_turn;
        v_turn_l_90 = v_turn;
        v_turn_180 = v_turn;
        v_turn_d_90 = v_turn;
        v_turn_45 = v_turn;
        v_turn_135 = v_turn;
        a_straight = a;
        a_d_straight = a;
    }

    TurnParameter(float v_straight_, float v_d_straight_, float v_turn_90_, float v_turn_l_90_, float v_turn_180_,
            float v_turn_d_90_, float v_turn_45_, float v_turn_135_, float a_straight_, float a_d_straight_) {
        v_straight = v_straight_;
        v_d_straight = v_d_straight_;
        v_turn_90 = v_turn_90_;
        v_turn_l_90 = v_turn_l_90_;
        v_turn_180 = v_turn_180_;
        v_turn_d_90 = v_turn_d_90_;
        v_turn_45 = v_turn_45_;
        v_turn_135 = v_turn_135_;
        a_straight = a_straight_;
        a_d_straight = a_d_straight_;
    }

    float getTurnV(turn_type_e turn_type) {
        switch (turn_type) {
            case STRAIGHT:
                return v_straight;
            case TURN_90:
                return v_turn_90;
            case TURN_180:
                return v_turn_180;
            case TURN_S2D_45:
                return v_turn_45;
            case TURN_S2D_135:
                return v_turn_135;
            case TURN_D2S_45:
                return v_turn_45;
            case TURN_D2S_135:
                return v_turn_135;
            case D_STRAIGHT:
                return v_d_straight;
            default:
                return 0.0;
        }
    }
};

inline void makeMinStepPath(uint16_t goal_x, uint16_t goal_y, Maze &maze, vector<Path> &path_vec) {

    enum direction_e dir = N;
    uint16_t p_x = 0;
    uint16_t p_y = 0;

    maze.makeFastestMap(goal_x, goal_y);
    path_vec.push_back(Path(STRAIGHT, 1, NO_TURN));
    p_y++;

    while ((p_x != goal_x) || (p_y != goal_y)) {
        //waitmsec(3000);
        printfAsync("%d %d\n", p_x, p_y);
        direction_e min_dir = maze.getMinDirection(p_x, p_y, dir);
        if (min_dir == dir) {
            path_vec.push_back(Path(STRAIGHT, 1, NO_TURN));
            path_vec.push_back(Path(STRAIGHT, 1, NO_TURN));
        }
        else {
            int8_t dir_diff = (int8_t) min_dir - (int8_t) dir;
            if (dir_diff == 6) dir_diff = -2;
            if (dir_diff == -6) dir_diff = 2;
            turn_dir_e turn_dir = (turn_dir_e) (SIGN(dir_diff));
            path_vec.push_back(Path(TURN_90, 1, turn_dir));
        }
        dir = min_dir;
        switch (dir) {
            case E:
                p_x++;
                break;
            case N:
                p_y++;
                break;
            case W:
                p_x--;
                break;
            case S:
                p_y--;
                break;
        }
    }

    path_vec.push_back(Path(STRAIGHT, 1, NO_TURN));
}

inline void jointStraightPath(vector<Path> &path_vec) {
    for (uint16_t i = 0; i < (uint16_t) path_vec.size(); i++) {
        printfAsync("-------%d turntype %d \n", i, path_vec[i].turn_type);
        //waitmsec(3000);
        if (i == (uint16_t) path_vec.size()) {
            break;
        }
        else if (path_vec[i].turn_type == STRAIGHT) {
            if (i + 1 == (uint16_t) path_vec.size()) break;
            while (path_vec[i + 1].turn_type == STRAIGHT) {
                path_vec.erase(path_vec.begin() + i + 1);
                printfAsync("          -- i=%d -- size=%d --block_num=%d \n", i, path_vec.size(), path_vec[i].block_num);
                path_vec[i].block_num++;
                if (i + 1 == (uint16_t) path_vec.size()) break;
            }
        }
    }
}

/*
 inline void jointStraightPath(vector<Path> &path_vec) {
 for (uint16_t i = 0; i < (uint16_t) path_vec.size(); i++) {
 printfAsync("-------%d turntype %d \n", i, path_vec[i].turn_type);
 //waitmsec(3000);
 if (i == (uint16_t) path_vec.size()) {
 break;
 } else if (path_vec[i].turn_type == STRAIGHT) {
 while (path_vec[i + 1].turn_type == STRAIGHT) {
 path_vec.erase(path_vec.begin() + i + 1);
 printfAsync("          -- i=%d -- size=%d --block_num=%d \n", i,
 path_vec.size(), path_vec[i].block_num);
 path_vec[i].block_num++;
 if (i + 1 == (uint16_t) path_vec.size())
 break;
 }
 }
 }
 }
 */

inline void playPath(TurnParameter turn_p, vector<Path> &path_vec) {
    EventList &events = EventList::getInstance();
    float v_pre = 0.0;
    float v_fol = 0.0;

    float x, v, a, v_max, dir;
    for (uint16_t i = 0; i < (uint16_t) path_vec.size(); i++) {
        v_pre = v_fol;
        switch (path_vec[i].turn_type) {
            case STRAIGHT:
                x = float(path_vec[i].block_num) * 0.09;
                a = turn_p.a_straight;
                v_max = turn_p.getTurnV(STRAIGHT);

                if (i + 1 == (uint16_t) path_vec.size()) v_fol = 0.1;
                else v_fol = turn_p.getTurnV((turn_type_e) path_vec[i + 1].turn_type);

                events.push(new Trape(x, v_max, v_pre, v_fol, a, true));
                break;
            case TURN_90:
                v = turn_p.v_turn_90;
                const float d_pre = 0.03031937846141211;
                const float d_fol = 0.03029937846158079;
                dir = SIGN((float )path_vec[i].turn_dir);
                events.push(new Trape(d_pre, v, v, v_fol, a, true));
                events.push(new Slalom_classic_90deg_no_straight(v, dir));
                events.push(new Trape(d_fol, v, v, v, a, false));



        }
    }
    events.push(new Stop(3000));

}

inline void HF_playPath(TurnParameter turn_p, vector<Path> &path_vec) {
    EventList &events = EventList::getInstance();
    float v_pre = 0.0;
    float v_fol = 0.0;

    float x, v, a, v_max, dir;
    for (uint16_t i = 0; i < (uint16_t) path_vec.size(); i++) {
        v_pre = v_fol;
        switch (path_vec[i].turn_type) {
            case STRAIGHT:
                x = float(path_vec[i].block_num) * 0.045;
                a = turn_p.a_straight;
                v_max = turn_p.getTurnV(STRAIGHT);

                if (i + 1 == (uint16_t) path_vec.size()) v_fol = 0.1;
                else v_fol = turn_p.getTurnV((turn_type_e) path_vec[i + 1].turn_type);

                events.push(new Trape(x, v_max, v_pre, v_fol, a, true));
                break;
            case TURN_90:
                v = turn_p.v_turn_90;
                const float d_pre = 0.00940729611;
                const float d_fol = 0.00938729611;
                dir = SIGN((float )path_vec[i].turn_dir);
                events.push(new Trape(d_pre, v, v, v_fol, a, true));
                events.push(new Slalom_half_90deg_no_straight(v, dir));
                events.push(new Trape(d_fol, v, v, v, a, false));

                break;
        }
    }
    events.push(new Stop(3000));

}

inline void playPathPivot(TurnParameter turn_p, vector<Path> &path_vec) {
    EventList &events = EventList::getInstance();
    float v_pre = 0.0;
    float v_fol = 0.0;

    float x, v, a, v_max, dir;
    for (uint16_t i = 0; i < (uint16_t) path_vec.size(); i++) {
        v_pre = v_fol;
        switch (path_vec[i].turn_type) {
            case STRAIGHT:
                x = float(path_vec[i].block_num) * 0.09;
                a = turn_p.a_straight;
                v_max = turn_p.getTurnV(STRAIGHT);

                if (i + 1 == (uint16_t) path_vec.size()) v_fol = 0.1;
                else v_fol = turn_p.getTurnV((turn_type_e) path_vec[i + 1].turn_type);

                events.push(new Trape(x, v_max, v_pre, v_fol, a, true));
                break;
            case TURN_90:
                v = turn_p.v_turn_90;
                a = turn_p.a_straight;
                dir = SIGN((float )path_vec[i].turn_dir);
                events.push(new Trape(0.09, v, v, 0.15, a, false));
                events.push(new Stop(50));
                events.push(new PivotTurn(90.0f * dir));
                events.push(new Stop(50));
                events.push(new Trape(0.09, v, 0.0, v, a, false));
                break;
        }
    }
    events.push(new Stop(3000));

}


inline void HF_playPathPivot(TurnParameter turn_p, vector<Path> &path_vec) {
    EventList &events = EventList::getInstance();
    float v_pre = 0.0;
    float v_fol = 0.0;

    float x, v, a, v_max, dir;
    for (uint16_t i = 0; i < (uint16_t) path_vec.size(); i++) {
        v_pre = v_fol;
        switch (path_vec[i].turn_type) {
            case STRAIGHT:
                x = float(path_vec[i].block_num) * 0.045;
                a = turn_p.a_straight;
                v_max = turn_p.getTurnV(STRAIGHT);

                if (i + 1 == (uint16_t) path_vec.size()) v_fol = 0.1;
                else v_fol = turn_p.getTurnV((turn_type_e) path_vec[i + 1].turn_type);

                events.push(new Trape(x, v_max, v_pre, v_fol, a, true));
                break;
            case TURN_90:
                v = turn_p.v_turn_90;
                a = turn_p.a_straight;
                dir = SIGN((float )path_vec[i].turn_dir);
                events.push(new Trape(0.045, v, v, 0.15, a, false));
                events.push(new Stop(50));
                events.push(new PivotTurn(90.0f * dir));
                events.push(new Stop(50));
                events.push(new Trape(0.045, v, 0.0, v, a, false));
                break;
        }
    }
    events.push(new Stop(3000));

}


inline void playPathDebug(TurnParameter turn_p, vector<Path> &path_vec) {
    printfAsync("==== playPathDebug ====\n");
    EventList &events = EventList::getInstance();
    float v_pre = 0.0;
    float v_fol = 0.0;

    float x, v, a, v_max, dir;
    for (uint16_t i = 0; i < (uint16_t) path_vec.size(); i++) {
        v_pre = v_fol;
        switch (path_vec[i].turn_type) {
            case STRAIGHT:
                x = float(path_vec[i].block_num) * 0.09;
                a = turn_p.a_straight;
                v_max = turn_p.getTurnV(STRAIGHT);

                if (i + 1 == (uint16_t) path_vec.size()) v_fol = 0.0;
                else v_fol = turn_p.getTurnV((turn_type_e) path_vec[i + 1].turn_type);

                //            events.push(new Trape(x, v_max, v_pre, v_fol, a, true));
                printfAsync("straight: / %.2f / %.2f / %.2f / %.2f / %.2f / \n", x, v_max, v_pre, v_fol, a);

                break;
            case TURN_90:
                v = turn_p.v_turn_90;
                dir = SIGN((float )path_vec[i].turn_dir);
                //            events.push(new Slalom_classic_90deg(v, dir));
                printfAsync("turn_90: / %.2f / %.2f /\n", v, dir);
                break;
        }
    }

    printfAsync("stop\n");
    printfAsync("==================\n");

}

inline void printPath(vector<Path> &path_vec) {
    printfAsync("==== path_vec ====\n");
    for (auto &path : path_vec) {
        path.print();
    }
    printfAsync("==================\n");
}

}

