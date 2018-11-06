/*
 * parameterManager.cpp
 *
 *  Created on: 2017/08/27
 *      Author: ryota
 */

#include <stdint.h>
#include <string>
#include <map>
#include <typeinfo>

#include "parameterManager.h"
#include "dataFlash.h"
#include "communication.h"
#include "timer.h"
#include "sound.h"

namespace peri = peripheral_RX71M;


namespace robot_object{


    void ParameterManager::init(){
        registration<float>(0, mass);
        registration<float>(1, dia_tire);
        registration<float>(2, tread);
        registration<float>(3, inertia);
        registration<float>(4, offset_torque_right);
        registration<float>(5, K_T_right);
        registration<float>(6, torque_scaler_right);
        registration<float>(7, circuit_res_right);
        registration<float>(8, offset_torque_left);
        registration<float>(9, K_T_left);
        registration<float>(10, torque_scaler_left);
        registration<float>(11, circuit_res_left);
        registration<float>(12, straight_v_P);
        registration<float>(13, straight_v_I);
        registration<float>(14, straight_v_D);
        registration<float>(15, straight_ang_v_P);
        registration<float>(16, straight_ang_v_I);
        registration<float>(17, straight_ang_v_D);
        registration<float>(18, straight_ang_P);
        registration<float>(19, straight_ang_I);
        registration<float>(20, straight_ang_D);
        registration<float>(21, pivot_v_P);
        registration<float>(22, pivot_v_I);
        registration<float>(23, pivot_v_D);
        registration<float>(24, pivot_ang_v_P);
        registration<float>(25, pivot_ang_v_I);
        registration<float>(26, pivot_ang_v_D);
        registration<float>(27, pivot_ang_P);
        registration<float>(28, pivot_ang_I);
        registration<float>(29, pivot_ang_D);
        registration<float>(30, slalom_v_P);
        registration<float>(31, slalom_v_I);
        registration<float>(32, slalom_v_D);
        registration<float>(33, slalom_ang_v_P);
        registration<float>(34, slalom_ang_v_I);
        registration<float>(35, slalom_ang_v_D);
        registration<float>(36, slalom_ang_P);
        registration<float>(37, slalom_ang_I);
        registration<float>(38, slalom_ang_D);
        registration<float>(39, wall_P);
        registration<float>(40, wall_I);
        registration<float>(41, wall_D);
        registration<float>(42, y_lc_P);
        registration<float>(43, y_lc_I);
        registration<float >(44, y_lc_D);
        registration<float>(45, HF_wall_P);
        registration<float>(46, HF_wall_I);
        registration<float>(47, HF_wall_D);


        registration<uint16_t>(100, front_wall_center_r);
        registration<uint16_t>(101, front_wall_center_l);
        registration<uint16_t>(102, back_wall_center_r);
        registration<uint16_t>(103, back_wall_center_l);
        registration<uint16_t>(104, front_collision_thr_ahead);
        registration<uint16_t>(105, back_collision_thr_ahead);
        registration<uint16_t>(106, front_wall_threshold_right);
        registration<uint16_t>(107, front_wall_threshold_left);
        registration<uint16_t>(108, front_wall_threshold_ahead);
        registration<uint16_t>(109, back_wall_threshold_right);
        registration<uint16_t>(110, back_wall_threshold_left);
        registration<uint16_t>(111, back_wall_threshold_ahead);
        registration<uint16_t>(112, front_wall_ctrl_threshold_right);
        registration<uint16_t>(113, front_wall_ctrl_threshold_left);
        registration<uint16_t>(114, back_wall_ctrl_threshold_right);
        registration<uint16_t>(115, back_wall_ctrl_threshold_left);
        registration<uint16_t>(116, front_wall_ctrl_threshold_delta_right);
        registration<uint16_t>(117, front_wall_ctrl_threshold_delta_left);
        registration<uint16_t>(118, front_wall_ctrl_add_val_right);
        registration<uint16_t>(119, front_wall_ctrl_add_val_left);
        registration<uint16_t>(120, back_wall_ctrl_threshold_delta_right);
        registration<uint16_t>(121, back_wall_ctrl_threshold_delta_left);
        registration<uint16_t>(122, back_wall_ctrl_add_val_right);
        registration<uint16_t>(123, back_wall_ctrl_add_val_left);
        registration<uint16_t>(124, HF_front_wall_center_r);
        registration<uint16_t>(125, HF_front_wall_center_l);
        registration<uint16_t>(126, HF_back_wall_center_r);
        registration<uint16_t>(127, HF_back_wall_center_l);
        registration<uint16_t>(128, HF_front_collision_thr_ahead);
        registration<uint16_t>(129, HF_back_collision_thr_ahead);
        registration<uint16_t>(130, HF_front_wall_threshold_right);
        registration<uint16_t>(131, HF_front_wall_threshold_left);
        registration<uint16_t>(132, HF_front_wall_threshold_ahead);
        registration<uint16_t>(133, HF_back_wall_threshold_right);
        registration<uint16_t>(134, HF_back_wall_threshold_left);
        registration<uint16_t>(135, HF_back_wall_threshold_ahead);
        registration<uint16_t>(136, HF_front_wall_ctrl_threshold_right);
        registration<uint16_t>(137, HF_front_wall_ctrl_threshold_left);
        registration<uint16_t>(138, HF_back_wall_ctrl_threshold_right);
        registration<uint16_t>(139, HF_back_wall_ctrl_threshold_left);
        registration<uint16_t>(140, HF_front_wall_ctrl_threshold_delta_right);
        registration<uint16_t>(141, HF_front_wall_ctrl_threshold_delta_left);
        registration<uint16_t>(142, HF_front_wall_ctrl_add_val_right);
        registration<uint16_t>(143, HF_front_wall_ctrl_add_val_left);
        registration<uint16_t>(144, HF_back_wall_ctrl_threshold_delta_right);
        registration<uint16_t>(145, HF_back_wall_ctrl_threshold_delta_left);
        registration<uint16_t>(146, HF_back_wall_ctrl_add_val_right);
        registration<uint16_t>(147, HF_back_wall_ctrl_add_val_left);

        registration<int16_t>(150, gyro_x_ref);
        registration<int16_t>(151, gyro_y_ref);
        registration<int16_t>(152, gyro_z_ref);
        registration<int16_t>(153, acc_x_ref);
        registration<int16_t>(154, acc_y_ref);
        registration<int16_t>(155, acc_z_ref);
        registration<float>(156, duty_limit);
        registration<uint8_t>(157, silent_flag);
        registration<uint8_t>(158, send_data_mode);
        registration<float>(159, test_run_v);
        registration<float>(160, test_run_a);
        registration<float>(161, test_run_x);
        registration<uint8_t>(162, test_run_wall_flag);
        registration<float>(163, v_search_run);
        registration<float>(164, HF_v_search_run);
        registration<uint8_t>(165, goal_x);
        registration<uint8_t>(166, goal_y);
        registration<uint8_t>(167, half_flag);
        registration<float>(168, a_search_run);
        registration<float>(169, HF_a_search_run);
        registration<float>(170, pivot_ang_v);
        registration<float>(171, pivot_ang_a);
    }

    void ParameterManager::setStrkey(uint16_t val_num, std::string key){
        strkeyMap[key] = val_num;
    }

    //プログラム中の変数にデータフラッシュの保存域を割り当て
    //登録時に変数にデータフラッシュに保存されている値を代入
    //登録を行った変数はデータフラッシュの保存域を更新する(write関数)際に値を共に変更
    template<typename T>
    void ParameterManager::registration(uint16_t val_num, T& r_val){
        uint16_t index = val_num *64;
        //uint8_t len = sizeof(T);
        T* adr = &r_val;
        adrMap[val_num] = reinterpret_cast<uint32_t>(adr);
        r_val = read<T>(val_num);

        if(typeid(float) == typeid(r_val)) typeMap[val_num] = Type_e::FLOAT;
        if(typeid(uint8_t) == typeid(r_val)) typeMap[val_num] = Type_e::UINT8;
        if(typeid(uint16_t) == typeid(r_val)) typeMap[val_num] = Type_e::UINT16;
        if(typeid(uint32_t) == typeid(r_val)) typeMap[val_num] = Type_e::UINT32;
        if(typeid(int8_t) == typeid(r_val)) typeMap[val_num] = Type_e::INT8;
        if(typeid(int16_t) == typeid(r_val)) typeMap[val_num] = Type_e::INT16;
        if(typeid(int32_t) == typeid(r_val)) typeMap[val_num] = Type_e::INT32;

    }

    template<typename T>
    bool ParameterManager::write(uint16_t val_num, T val){
        uint16_t index = val_num * 64;
        bool rtn;
        while(1){
            if(peri::eraseCheckDataFlash(index, 64) == false){
                peri::eraseDataFlash(index);
            };
            rtn = peri::writeDataFlash(index, &val, sizeof(T));
            printfAsync("write error!\n");
            if(read<T>(val_num) == val) break;
        }

        //val_numに変数が登録されている場合はその変数を書き換え
        if (adrMap.find(val_num) != adrMap.end() ) {
            *reinterpret_cast<T*>(adrMap[val_num]) = val;
            printfAsync("write: %f %f \n",val, *reinterpret_cast<T*>(adrMap[val_num]));
        }
        return rtn;
    }

    template<typename T>
    T ParameterManager::read(uint16_t val_num){
        T val;
        uint16_t index = val_num * 64;
        peri::readDataFlash(index, &val, sizeof(T));

        return val;
    };

    template<typename T>
    bool ParameterManager::write(std::string key, T val){
        return write(strkeyMap[key], val);
    }

    template<typename T>
    T ParameterManager::read(std::string key){
        return read<T>(strkeyMap[key]);
    };

    bool ParameterManager::writeCommand(uint8_t *command){
        uint8_t chk_sum = 0;
        for(uint8_t i=5;i<16;i++) chk_sum+= command[i];
        if(chk_sum == command[4]){
        	SED();  //音を鳴らす
            switch(command[5]){
                case 0:
                    write<float>(command[10], *((float*)&command[6]));
                    printfAsync("%d %f %f \n",command[10], *((float*)&command[6]) ,read<float>(command[10]));
                    break;
                case 1:
                    write<uint8_t>(command[10], command[6]);
                    printfAsync("%d \n",*((uint8_t*)&command[6]));
                    break;
                case 2:
                    write<uint16_t>(command[10], *((uint16_t*)&command[6]));
                    printfAsync("%d \n",*((uint16_t*)&command[6]));
                    break;
                case 3:
                    write<uint32_t>(command[10], *((uint32_t*)&command[6]));
                    printfAsync("%d \n",*((uint32_t*)&command[6]));
                    break;
                case 4:
                    write<int8_t>(command[10], *((int8_t*)&command[6]));
                    printfAsync("%d \n",*((int8_t*)&command[6]));
                    break;
                case 5:
                    write<int16_t>(command[10], *((int16_t*)&command[6]));
                    printfAsync("%d | %d %d  \n",command[10], *((int16_t*)&command[6]), read<int16_t>(command[10]) );

                    break;
                case 6:
                    write<int32_t>(command[10], *((int32_t*)&command[6]));
                    printfAsync("%d \n",*((int32_t*)&command[6]));
                    break;

            }
        }

    }


    //テンプレートクラスの実体化
    template void ParameterManager::registration(uint16_t val_num, float& r_val);
    template void ParameterManager::registration(uint16_t val_num, uint8_t& r_val);
    template void ParameterManager::registration(uint16_t val_num, uint16_t& r_val);
    template void ParameterManager::registration(uint16_t val_num, uint32_t& r_val);
    template void ParameterManager::registration(uint16_t val_num, int8_t& r_val);
    template void ParameterManager::registration(uint16_t val_num, int16_t& r_val);
    template void ParameterManager::registration(uint16_t val_num, int32_t& r_val);

    template bool ParameterManager::write(uint16_t val_num, float val);
    template bool ParameterManager::write(uint16_t val_num, uint8_t val);
    template bool ParameterManager::write(uint16_t val_num, uint16_t val);
    template bool ParameterManager::write(uint16_t val_num, uint32_t val);
    template bool ParameterManager::write(uint16_t val_num, int8_t val);
    template bool ParameterManager::write(uint16_t val_num, int16_t val);
    template bool ParameterManager::write(uint16_t val_num, int32_t val);

    template float ParameterManager::read<float>(uint16_t val_num);
    template uint8_t ParameterManager::read<uint8_t>(uint16_t val_num);
    template uint16_t ParameterManager::read<uint16_t>(uint16_t val_num);
    template uint32_t ParameterManager::read<uint32_t>(uint16_t val_num);
    template int8_t ParameterManager::read<int8_t>(uint16_t val_num);
    template int16_t ParameterManager::read<int16_t>(uint16_t val_num);
    template int32_t ParameterManager::read<int32_t>(uint16_t val_num);

    template bool ParameterManager::write(std::string key, float val);
    template bool ParameterManager::write(std::string key, uint8_t val);
    template bool ParameterManager::write(std::string key, uint16_t val);
    template bool ParameterManager::write(std::string key, uint32_t val);
    template bool ParameterManager::write(std::string key, int8_t val);
    template bool ParameterManager::write(std::string key, int16_t val);
    template bool ParameterManager::write(std::string key, int32_t val);

    template float ParameterManager::read<float>(std::string key);
    template uint8_t ParameterManager::read<uint8_t>(std::string key);
    template uint16_t ParameterManager::read<uint16_t>(std::string key);
    template uint32_t ParameterManager::read<uint32_t>(std::string key);
    template int8_t ParameterManager::read<int8_t>(std::string key);
    template int16_t ParameterManager::read<int16_t>(std::string key);
    template int32_t ParameterManager::read<int32_t>(std::string key);




}




