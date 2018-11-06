/*
 * pwm.cpp
 *
 *  Created on: 2016/11/06
 *      Author: ryota
 */

#include <myUtil.h>
#include "iodefine.h"

#include <stdint.h>

#include "pwm.h"
#include "clock.h"

namespace peripheral_RX71M{

    static float dutyMTU0;
    static float dutyMTU3;
    static float dutyMTU4;
    static float dutyMTU7;
    /////////////////////////////////////////////////////////////
    void initMTU0(){
        SYSTEM.PRCR.WORD = 0xA502;
        SYSTEM.MSTPCRA.BIT.MSTPA9  = 0;//MTUモジュールON
        SYSTEM.PRCR.WORD = 0xA500;

        MPC.PWPR.BIT.B0WI=0;
        MPC.PWPR.BIT.PFSWE=1;
        MPC.PB3PFS.BIT.PSEL=1;//MTIOC0A
        MPC.PWPR.BYTE=0x80;

        PORTB.PMR.BIT.B3=1;

        MTU0.TCR.BIT.TPSC=0;//PCLKA/1
        MTU0.TCR.BIT.CCLR=1;//PWM TGRAのコンペアマッチでTCNTクリア TGRDは6
        MTU0.TIORH.BIT.IOA=1;//初期出力0ンペアマッチ0出力
        MTU0.TIORH.BIT.IOB=2;//初期出力0コンペアマッチ1出力
        MTU0.TGRA = 4800;
        MTU0.TGRB = 1000;
        MTU0.TGRC = 4800;
        MTU0.TGRD = 1000;
        MTU0.TMDR1.BIT.MD=2;//PWM1
        //MTU3.TMDR1.BIT.BFA = 1;   //バッファーモードに設定
        MTU0.TMDR1.BIT.BFB = 1;
    }


    /////////////////////////////////////////////////////////////
    void initMTU3(){
        SYSTEM.PRCR.WORD = 0xA502;
        SYSTEM.MSTPCRA.BIT.MSTPA9  = 0;//MTUモジュールON
        SYSTEM.PRCR.WORD = 0xA500;

        MPC.PWPR.BIT.B0WI=0;
        MPC.PWPR.BIT.PFSWE=1;
        MPC.P14PFS.BIT.PSEL=1;//MTIOC3A
        MPC.PWPR.BYTE=0x80;

        PORT1.PMR.BIT.B4=1;

        MTU3.TCR.BIT.TPSC=0;//PCLKA/1
        MTU3.TCR.BIT.CCLR=1;//PWM TGRAのコンペアマッチでTCNTクリア TGRDは6
        MTU3.TIORH.BIT.IOA=1;//初期出力0ンペアマッチ0出力
        MTU3.TIORH.BIT.IOB=2;//初期出力0コンペアマッチ1出力
        MTU3.TGRA = 4800;
        MTU3.TGRB = 1000;
        MTU3.TGRC = 4800;
        MTU3.TGRD = 1000;
        MTU3.TMDR1.BIT.MD=2;//PWM1
        //MTU3.TMDR1.BIT.BFA = 1;	//バッファーモードに設定
        MTU3.TMDR1.BIT.BFB = 1;
    }


    /////////////////////////////////////////////////////////////
    void initMTU4(){
        SYSTEM.PRCR.WORD = 0xA502;
        SYSTEM.MSTPCRA.BIT.MSTPA9  = 0;//MTUモジュールON
        SYSTEM.PRCR.WORD = 0xA500;

        MPC.PWPR.BIT.B0WI=0;
        MPC.PWPR.BIT.PFSWE=1;
        MPC.P21PFS.BIT.PSEL=0b001000;//MTIOC4A
        MPC.PWPR.BYTE=0x80;

        PORT2.PMR.BIT.B1=1;//左PWM
        MTU.TSTRA.BIT.CST4 = 1;
        MTU.TOERA.BIT.OE4A=1;//MTU出力端子を出力許可する

        MTU4.TCR.BIT.TPSC=0;//PCLKA/1
        MTU4.TCR.BIT.CCLR=1;//PWM TGRAのコンペアマッチでTCNTクリア TGRDは6
        MTU4.TIORH.BIT.IOA=1;//初期出力0ンペアマッチ0出力
        MTU4.TIORH.BIT.IOB=2;//初期出力0コンペアマッチ1出力
        MTU4.TGRA = 4800;
        MTU4.TGRB = 1000;
        MTU4.TGRC = 4800;
        MTU4.TGRD = 1000;
        MTU4.TMDR1.BIT.MD=2;//PWM1
        //MTU4.TMDR1.BIT.BFA = 1;	//バッファーモードに設定
        MTU4.TMDR1.BIT.BFB = 1;	//バッファーモードに設定
    }

    /////////////////////////////////////////////////////////////
     void initMTU7(){
         SYSTEM.PRCR.WORD = 0xA502;
         SYSTEM.MSTPCRA.BIT.MSTPA9  = 0;//MTUモジュールON
         SYSTEM.PRCR.WORD = 0xA500;

         MPC.PWPR.BIT.B0WI=0;
         MPC.PWPR.BIT.PFSWE=1;
         MPC.PA2PFS.BIT.PSEL=8;//MTIOC7A
         MPC.PWPR.BYTE=0x80;

         PORTA.PMR.BIT.B2=1;

         MTU7.TCR.BIT.TPSC=0;//PCLKA/1
         MTU7.TCR.BIT.CCLR=1;//PWM TGRAのコンペアマッチでTCNTクリア TGRDは6
         MTU7.TIORH.BIT.IOA=1;//初期出力0ンペアマッチ0出力
         MTU7.TIORH.BIT.IOB=2;//初期出力0コンペアマッチ1出力
         MTU7.TGRA = 4800;
         MTU7.TGRB = 1000;
         MTU7.TGRC = 4800;
         MTU7.TGRD = 1000;
         MTU7.TMDR1.BIT.MD=2;//PWM1
         //MTU7.TMDR1.BIT.BFA = 1;   //バッファーモードに設定
         MTU7.TMDR1.BIT.BFB = 1;
         MTU.TOERB.BIT.OE7A=1;//MTU出力端子を出力許可する
     }

     void setDutyMTU0(float duty){
         duty = constrain(duty, 0.0, 1.0);
         dutyMTU0 = duty;
         if(duty == 0.0){
             MTU.TSTRA.BIT.CST0 = 0;
             return;
         }
         MTU0.TGRD = (uint16_t)(MTU0.TGRC * (1.0-duty));
         MTU.TSTRA.BIT.CST0 = 1;
    }


    void setDutyMTU3(float duty){
        duty = constrain(duty, 0.0, 1.0);
        dutyMTU3 = duty;
        if(duty == 0.0){
            MTU.TSTRA.BIT.CST3 = 0;
            return;
        }
        MTU3.TGRD = (uint16_t)(MTU3.TGRC * (1.0-duty));
        MTU.TSTRA.BIT.CST3 = 1;
    }

    void setDutyMTU4(float duty){
        duty = constrain(duty, 0.0, 1.0);
        dutyMTU4 = duty;
        if(duty == 0.0){
            MTU.TSTRA.BIT.CST4 = 0;
            return;
        }
        MTU4.TGRD = (uint16_t)(MTU4.TGRC * (1.0-duty));
        MTU.TSTRA.BIT.CST4 = 1;
    }

    void setDutyMTU7(float duty){
        duty = constrain(duty, 0.0, 1.0);
        dutyMTU7 = duty;
        if(duty == 0.0){
            MTU.TSTRB.BIT.CST7 = 0;
            return;
        }
        MTU7.TGRD = (uint16_t)(MTU7.TGRC * (1.0-duty));
        MTU.TSTRB.BIT.CST7 = 1;
   }


    float getDutyMTU0(){
        return dutyMTU0;
    }

    float getDutyMTU3(){
        return dutyMTU3;
    }

    float getDutyMTU4(){
        return dutyMTU4;
    }

    float getDutyMTU7(){
        return dutyMTU7;
    }


}


