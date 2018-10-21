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

        uint8_t goal_x; //100
        uint8_t goal_y; //101
        uint8_t half_flag; //102
        uint16_t front_wall_center_r; //103
        uint16_t front_wall_center_l; //104
        uint16_t back_wall_center_r; //105
        uint16_t back_wall_center_l; //106
        uint16_t front_collision_thr_ahead; //107
        uint16_t back_collision_thr_ahead; //108
        uint16_t front_wall_threshold_right; //109
        uint16_t front_wall_threshold_left; //110
        uint16_t front_wall_threshold_ahead; //111
        uint16_t back_wall_threshold_right; //112
        uint16_t back_wall_threshold_left; //113
        uint16_t back_wall_threshold_ahead; //114
        uint16_t HF_front_wall_center_r; //115
        uint16_t HF_front_wall_center_l; //116
        uint16_t HF_back_wall_center_r; //117
        uint16_t HF_back_wall_center_l; //118
        uint16_t HF_front_collision_thr_ahead; //119
        uint16_t HF_back_collision_thr_ahead; //120
        uint16_t HF_front_wall_threshold_right; //121
        uint16_t HF_front_wall_threshold_left; //122
        uint16_t HF_front_wall_threshold_ahead; //123
        uint16_t HF_back_wall_threshold_right; //124
        uint16_t HF_back_wall_threshold_left; //125
        uint16_t HF_back_wall_threshold_ahead; //126
        float v_search_run; //127
        float HF_v_search_run; //128

        int16_t gyro_x_ref; //150
        int16_t gyro_y_ref; //151
        int16_t gyro_z_ref; //152
        int16_t acc_x_ref; //153
        int16_t acc_y_ref; //154
        int16_t acc_z_ref; //155
        float duty_limit; //156
        uint8_t silent_flag; //157
        uint8_t send_data_mode; //158
        //----管理下においた変数たち-----



    private:

        ParameterManager(){};
        ~ParameterManager(){};
        ParameterManager(ParameterManager&){};
    };

}



