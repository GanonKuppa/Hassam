/**
 * @file   clock.cpp
 * @brief  GPIOピンの設定
 *
 * @date 2016/7/23
 * @author ryota
 */

#include "iodefine.h"
#include "gpio.h"
#include <stdint.h>

namespace peripheral_RX71M{
    void initGPIO(){
        //未使用ピンの処理
        PORT0.PDR.BYTE = (uint8_t)(PORT0.PDR.BYTE | 0x0F);
        PORT1.PDR.BYTE = (uint8_t)(PORT1.PDR.BYTE | 0x03);
        PORT5.PDR.BYTE = (uint8_t)(PORT5.PDR.BYTE | 0x40);
        PORT6.PDR.BYTE = (uint8_t)(PORT6.PDR.BYTE | 0xFF);
        PORT7.PDR.BYTE = (uint8_t)(PORT7.PDR.BYTE | 0xFF);
        PORT8.PDR.BYTE = (uint8_t)(PORT8.PDR.BYTE | 0xCF);
        PORT9.PDR.BYTE = (uint8_t)(PORT9.PDR.BYTE | 0xFF);
        PORTF.PDR.BYTE = (uint8_t)(PORTF.PDR.BYTE | 0x3F);
        PORTG.PDR.BYTE = (uint8_t)(PORTG.PDR.BYTE | 0xFF);
        PORTJ.PDR.BYTE = (uint8_t)(PORTJ.PDR.BYTE | 0x20);

        //FCLED
        PORTB.PDR.BIT.B4 = 1; //R
        PORTC.PDR.BIT.B0 = 1; //G
        PORTC.PDR.BIT.B1 = 1; //B

        //SWITCH
        PORTB.PDR.BIT.B5 = 0;
        PORTB.PCR.BIT.B5 = 1; //プルアップ有効

        //motor1
        PORT2.PDR.BIT.B1 = 1; //MOTOR_PWM_1 MTIOC4A
        PORT2.PDR.BIT.B0 = 1; //MOTOR_IN1_1
        PORT1.PDR.BIT.B7 = 1; //MOTOR_IN2_1
        //motor2
        PORT1.PDR.BIT.B4 = 1; //MOTOR_PWM_2 MTIOC3A
        PORT4.PDR.BIT.B1 = 1; //MOTOR_IN1_2
        PORT4.PDR.BIT.B2 = 1; //MOTOR_IN2_2
        //motor3
        PORTB.PDR.BIT.B3 = 1; //MOTOR_PWM_3 MTIOC0A
        PORTB.PDR.BIT.B1 = 1; //MOTOR_IN1_3
        PORTB.PDR.BIT.B2 = 1; //MOTOR_IN2_3
        //motor4
        PORTA.PDR.BIT.B2 = 1; //MOTOR_PWM_4 MTIOC7A
        PORTA.PDR.BIT.B1 = 1; //MOTOR_IN1_4
        PORTA.PDR.BIT.B3 = 1; //MOTOR_IN2_4

        //motor1
        PORT2.PODR.BIT.B1 = 0; //MOTOR_PWM_1 MTIOC4A
        PORT2.PODR.BIT.B0 = 0; //MOTOR_IN1_1
        PORT1.PODR.BIT.B7 = 0; //MOTOR_IN2_1
        //motor2
        PORT1.PODR.BIT.B4 = 0; //MOTOR_PWM_2 MTIOC3A
        PORT4.PODR.BIT.B1 = 0; //MOTOR_IN1_2
        PORT4.PODR.BIT.B2 = 0; //MOTOR_IN2_2
        //motor3
        PORTB.PODR.BIT.B3 = 0; //MOTOR_PWM_3 MTIOC0A
        PORTB.PODR.BIT.B1 = 0; //MOTOR_IN1_3
        PORTB.PODR.BIT.B2 = 0; //MOTOR_IN2_3
        //motor4
        PORTA.PODR.BIT.B2 = 0; //MOTOR_PWM_4 MTIOC7A
        PORTA.PODR.BIT.B1 = 0; //MOTOR_IN1_4
        PORTA.PODR.BIT.B3 = 0; //MOTOR_IN2_4


        //センサLED
        PORTE.PDR.BIT.B2 = 1; //SLED_OUT1
        PORTD.PDR.BIT.B6 = 1; //SLED_OUT2
        PORTD.PDR.BIT.B7 = 1; //SLED_OUT3
        PORT1.PDR.BIT.B2 = 1; //SLED_OUT4 配線ミスを繋ぎ直し
        PORTC.PDR.BIT.B2 = 1; //SLED_OUT5
        PORTC.PDR.BIT.B3 = 1; //SLED_OUT6

        //センサLED消灯
        PORTE.PODR.BIT.B2 = 0; //SLED_OUT1
        PORTD.PODR.BIT.B6 = 0; //SLED_OUT2
        PORTD.PODR.BIT.B7 = 0; //SLED_OUT3
        PORT1.PODR.BIT.B2 = 0; //SLED_OUT4 配線ミスを繋ぎ直し
        PORTC.PODR.BIT.B2 = 0; //SLED_OUT5
        PORTC.PODR.BIT.B3 = 0; //SLED_OUT6


    }
}
