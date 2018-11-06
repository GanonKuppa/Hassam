/*
 * parameterManager.h
 *
 *  Created on: 2017/08/27
 *      Author: ryota
 */

#pragma once

#include<stdint.h>
#include<string>
#include<map>


namespace robot_object{

    enum struct Type_e{
        FLOAT =0,
        UINT8 ,
        UINT16,
        UINT32,
        INT8,
        INT16,
        INT32
    };

    class ParameterManager{
    public:
        std::map<uint16_t, uint32_t> adrMap;
        std::map<uint16_t, Type_e> typeMap;
        std::map<std::string,uint16_t> strkeyMap;

        static ParameterManager& getInstance(){
            static ParameterManager instance;
            return instance;
        }

        void init();

        template<typename T>
        bool write(uint16_t val_num, T val);

        template<typename T>
        T read(uint16_t val_num);

        template<typename T>
        bool write(std::string key, T val);

        template<typename T>
        T read(std::string key);


        template<typename T>
        void registration(uint16_t val_num, T& r_val);
        void setStrkey(uint16_t val_num, std::string key);

        bool writeCommand(uint8_t *commnd);

        //----管理下においた変数たち-----
        float mass; //0
        float dia_tire; //1
        float tread; //2
        float inertia; //3
        float offset_torque_right; //4
        float K_T_right; //5
        float torque_scaler_right; //6
        float circuit_res_right; //7
        float offset_torque_left; //8
        float K_T_left; //9
        float torque_scaler_left; //10
        float circuit_res_left; //11
        float straight_v_P; //12
        float straight_v_I; //13
        float straight_v_D; //14
        float straight_ang_v_P; //15
        float straight_ang_v_I; //16
        float straight_ang_v_D; //17
        float straight_ang_P; //18
        float straight_ang_I; //19
        float straight_ang_D; //20
        float pivot_v_P; //21
        float pivot_v_I; //22
        float pivot_v_D; //23
        float pivot_ang_v_P; //24
        float pivot_ang_v_I; //25
        float pivot_ang_v_D; //26
        float pivot_ang_P; //27
        float pivot_ang_I; //28
        float pivot_ang_D; //29
        float slalom_v_P; //30
        float slalom_v_I; //31
        float slalom_v_D; //32
        float slalom_ang_v_P; //33
        float slalom_ang_v_I; //34
        float slalom_ang_v_D; //35
        float slalom_ang_P; //36
        float slalom_ang_I; //37
        float slalom_ang_D; //38
        float wall_P; //39
        float wall_I; //40
        float wall_D; //41
        float y_lc_P; //42
        float y_lc_I; //43
        float  y_lc_D; //44
        float HF_wall_P; //45
        float HF_wall_I; //46
        float HF_wall_D; //47

        uint16_t front_wall_center_r; //100
        uint16_t front_wall_center_l; //101
        uint16_t back_wall_center_r; //102
        uint16_t back_wall_center_l; //103
        uint16_t front_collision_thr_ahead; //104
        uint16_t back_collision_thr_ahead; //105
        uint16_t front_wall_threshold_right; //106
        uint16_t front_wall_threshold_left; //107
        uint16_t front_wall_threshold_ahead; //108
        uint16_t back_wall_threshold_right; //109
        uint16_t back_wall_threshold_left; //110
        uint16_t back_wall_threshold_ahead; //111
        uint16_t front_wall_ctrl_threshold_right; //112
        uint16_t front_wall_ctrl_threshold_left; //113
        uint16_t back_wall_ctrl_threshold_right; //114
        uint16_t back_wall_ctrl_threshold_left; //115
        uint16_t front_wall_ctrl_threshold_delta_right; //116
        uint16_t front_wall_ctrl_threshold_delta_left; //117
        uint16_t front_wall_ctrl_add_val_right; //118
        uint16_t front_wall_ctrl_add_val_left; //119
        uint16_t back_wall_ctrl_threshold_delta_right; //120
        uint16_t back_wall_ctrl_threshold_delta_left; //121
        uint16_t back_wall_ctrl_add_val_right; //122
        uint16_t back_wall_ctrl_add_val_left; //123
        uint16_t HF_front_wall_center_r; //124
        uint16_t HF_front_wall_center_l; //125
        uint16_t HF_back_wall_center_r; //126
        uint16_t HF_back_wall_center_l; //127
        uint16_t HF_front_collision_thr_ahead; //128
        uint16_t HF_back_collision_thr_ahead; //129
        uint16_t HF_front_wall_threshold_right; //130
        uint16_t HF_front_wall_threshold_left; //131
        uint16_t HF_front_wall_threshold_ahead; //132
        uint16_t HF_back_wall_threshold_right; //133
        uint16_t HF_back_wall_threshold_left; //134
        uint16_t HF_back_wall_threshold_ahead; //135
        uint16_t HF_front_wall_ctrl_threshold_right; //136
        uint16_t HF_front_wall_ctrl_threshold_left; //137
        uint16_t HF_back_wall_ctrl_threshold_right; //138
        uint16_t HF_back_wall_ctrl_threshold_left; //139
        uint16_t HF_front_wall_ctrl_threshold_delta_right; //140
        uint16_t HF_front_wall_ctrl_threshold_delta_left; //141
        uint16_t HF_front_wall_ctrl_add_val_right; //142
        uint16_t HF_front_wall_ctrl_add_val_left; //143
        uint16_t HF_back_wall_ctrl_threshold_delta_right; //144
        uint16_t HF_back_wall_ctrl_threshold_delta_left; //145
        uint16_t HF_back_wall_ctrl_add_val_right; //146
        uint16_t HF_back_wall_ctrl_add_val_left; //147

        int16_t gyro_x_ref; //150
        int16_t gyro_y_ref; //151
        int16_t gyro_z_ref; //152
        int16_t acc_x_ref; //153
        int16_t acc_y_ref; //154
        int16_t acc_z_ref; //155
        float duty_limit; //156
        uint8_t silent_flag; //157
        uint8_t send_data_mode; //158
        float test_run_v; //159
        float test_run_a; //160
        float test_run_x; //161
        uint8_t test_run_wall_flag; //162
        float v_search_run; //163
        float HF_v_search_run; //164
        uint8_t goal_x; //165
        uint8_t goal_y; //166
        uint8_t half_flag; //167
        float a_search_run; //168
        float HF_a_search_run; //169
        float pivot_ang_v; //170
        float pivot_ang_a; //171
        //----管理下においた変数たち-----



    private:

        ParameterManager(){};
        ~ParameterManager(){};
        ParameterManager(ParameterManager&){};
    };

}



