#pragma once

#include <stdint.h>
#include <math.h>
#include <myUtil.hpp>

#include "iodefine.h"
#include "pwm.h"
#include "phaseCounting.h"

#include "maze.h"
#include "machineParameters.h"
#include "math.h"
#include "imu.hpp"
#include "parameterManager.h"
#include "timer.h"
#include "wheelOdometry.hpp"
#include "ICM20602.hpp"

namespace peri = peripheral_RX71M;
using peri:: waitmsec;

//#include <Core>
//#include <Geometry>
//using namespace Eigen;
//#define EIGEN_DONT_VECTORIZE
//#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT


namespace robot_object {

    template <typename T>
    class Coor2D {
    public:
        T x;
        T y;

        bool operator==(const Coor2D& coor){
            if(coor.x == x && coor.y == y) return true;
            else return false;
        }
        void set(T x_, T y_){
            x = x_;
            y = y_;
        }

    };

    class Vector2f {
    public:
        float x;
        float y;

        Vector2f(float x_0, float y_0){
            x = x_0;
            y = y_0;
        }

        Vector2f operator +(Vector2f obj){
            obj.x += x;
            obj.y += y;
            return obj;
        }

        Vector2f operator -(Vector2f obj){
            obj.x -= x;
            obj.y -= y;
            return obj;
        }

        float operator [](int n){
            if(n == 0) return x;
            if(n == 1) return y;
            return 0.0;
        }

    };


    class UMouse {
    public:
        // -32768 から 32767

        volatile float t_a;       //event側で更新     x 1000
        volatile float t_v;       //event側で更新     x 3000
        volatile float t_x;       //event側で更新     x 3000
        volatile float accum_x;   //event側で更新     x 3000

        volatile float t_ang_a;   //event側で更新    x 100
        volatile float t_ang_v;   //event側で更新    x 100
        volatile float t_ang;     //event側で更新    x 100
        volatile float accum_ang; //event側で更新    x 100
        volatile float gyro_ang_v;// -2000deg/secから+2000deg/sec
        volatile float acc_v;

        volatile float wall_P;    //event側で更新  -10.0から10.0   x 3000
        volatile float wall_I;    //event側で更新  -10.0から10.0   x 3000
        volatile float wall_D;    //event側で更新  -10.0から10.0   x 3000

        volatile float v_P;       //event側で更新  -10.0から10.0   x 3000
        volatile float v_I;       //event側で更新  -10.0から10.0   x 3000
        volatile float v_D;       //event側で更新  -10.0から10.0   x 3000

        volatile float ang_v_P;   //event側で更新  -10.0から10.0   x 3000
        volatile float ang_v_I;   //event側で更新  -10.0から10.0   x 3000
        volatile float ang_v_D;   //event側で更新  -10.0から10.0   x 3000

        volatile float ang_P;     //event側で更新  -10.0から10.0   x 3000
        volatile float ang_I;     //event側で更新  -10.0から10.0   x 3000
        volatile float ang_D;     //event側で更新  -10.0から10.0   x 3000

        volatile float v_FF;      //event側で更新  -1.0から1.0     x 3000
        volatile float ang_FF;    //event側で更新  -1.0から1.0     x 3000

        volatile float duty_L;
        volatile float duty_R;

        direction_e direction;
        Coor2D<uint16_t> coor;
        Coor2D<uint16_t> start;
        Coor2D<uint16_t> goal;

        volatile float ang;
        volatile float ang_v;
        volatile float ang_a;

        Coor2D <float> v_g;
        Coor2D <float> x_g;
        Coor2D <float> a_g;

        Maze maze;

        bool switch_back; // 0:スイッチバックしてない状態 1:スイッチバックしている状態

        static UMouse& getInstance() {
            static UMouse instance;
            return instance;
        }



        void update(){
            WheelOdometry &wo = WheelOdometry::getInstance();
            ICM20602 &icm = ICM20602::getInstance();
            if(ABS(wo.v) < 0.01 ) acc_v =0.0;
            else acc_v += icm.acc_f[1] * 0.001;
        }

        void setDuty(float duty_l, float duty_r){
            if (switch_back == false){
                setDuty_L(duty_l);
                setDuty_R(duty_r);
            }
            else{
                setDuty_L(-duty_r);
                setDuty_R(-duty_l);

            }
        }

        void setDuty_R(float duty){
            ParameterManager &pm = ParameterManager::getInstance();
            float abs_duty = constrain(ABS(duty), 0.0, pm.duty_limit);
            duty_R = SIGN(duty) * abs_duty;

            if(duty == 0.0){
                PORT2.PODR.BIT.B0 = 0;
                PORT1.PODR.BIT.B7 = 0;
            }
            else if(duty > 0){
                PORT2.PODR.BIT.B0 = 1;
                PORT1.PODR.BIT.B7 = 0;
            }
            else{
                PORT2.PODR.BIT.B0 = 0;
                PORT1.PODR.BIT.B7 = 1;
            }
            peri::setDutyMTU4(abs_duty);
        };
        void setDuty_L(float duty){
            ParameterManager &pm = ParameterManager::getInstance();
            float abs_duty = constrain(ABS(duty), 0.0, pm.duty_limit);
            duty_L = SIGN(duty) * abs_duty;

            if(duty == 0.0){
                PORTB.PODR.BIT.B1 = 0;
                PORTB.PODR.BIT.B2 = 0;
            }
            else if(duty > 0){
                PORTB.PODR.BIT.B1 = 1;
                PORTB.PODR.BIT.B2 = 0;
            }
            else{
                PORTB.PODR.BIT.B1 = 0;
                PORTB.PODR.BIT.B2 = 1;
            }
            peri::setDutyMTU0(abs_duty);
        };



        void debug(){
            setDuty(0.1,0.1);
            waitmsec(500);
            setDuty(0.0, 0.0);
            waitmsec(4000);
            setDuty(0.2,0.2);
            waitmsec(500);
            setDuty(0.0,0.0);
            waitmsec(4000);
            setDuty(0.3,0.3);
            waitmsec(500);
            setDuty(0.0,0.0);
            waitmsec(4000);

            setDuty(-0.1,-0.1);
            waitmsec(500);
            setDuty(-0.0, -0.0);
            waitmsec(500);
            setDuty(-0.2,-0.2);
            waitmsec(500);
            setDuty(0.0,0.0);
            waitmsec(4000);
            setDuty(-0.3,-0.3);
            waitmsec(500);
            setDuty(0.0,0.0);
            waitmsec(500);

            setDuty(0.1,-0.1);
            waitmsec(1000);
            setDuty(0.0, -0.0);
            waitmsec(4000);
            setDuty(0.2,-0.2);
            waitmsec(1000);
            setDuty(0.0,0.0);
            waitmsec(4000);
            setDuty(0.3,-0.3);
            waitmsec(1000);
            setDuty(0.0,0.0);
            waitmsec(4000);

            setDuty(-0.1, 0.1);
            waitmsec(1000);
            setDuty(0.0, 0.0);
            waitmsec(4000);
            setDuty(-0.2, 0.2);
            waitmsec(1000);
            setDuty(0.0,0.0);
            waitmsec(4000);
            setDuty(-0.3, 0.3);
            waitmsec(1000);
            setDuty(0.0,0.0);
            waitmsec(4000);


        }


    private:
        UMouse() {

            direction = N;
            //ab_position.x = 0.09;
            //ab_position.y = 0.09;
            //ab_ang = 90.0;
            switch_back = false;
        };
        ~UMouse() {};

    };

}

