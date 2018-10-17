#ifndef MACHINEPARAMETERS_H
#define MACHINEPARAMETERS_H

/**************定数*****************************/

/**********マシン固有定数(ソフト)***************/
//#define DELTA_T (0.001)            //制御時間間隔
/**********マシン固有定数(メカ)**************/
#define TREAD      (0.068)           //トレッド幅
#define DIA_TIRE   (0.02437)          //タイヤ直径 小さくすると長く
#define INERTIA    (MASS*(0.08*0.08+0.025*0.025)/12.0) //9.040583333333333e-6
                                     //計算すると0.000953238 (MASS = 0.220)
                                     //慣性モーメント  90mm x  70mmの長方形と思って算出
#define MASS       (0.1421)         //質量
//#define GEAR_RATIO (3.66666666667)   //66:18 = 3.66…:1
//#define K_E        (0.000207)        //逆起電力定数[V/rpm]
//#define K_T        (0.00198)         //トルク定数[Nm/A]
#define COIL_RES   (1.07)            //巻き線抵抗[Ω]
#define FET_RES    (1.0)             //モータードライバのon抵抗[Ω]
#define CIRCUIT_RES (0.5)            //モーター配線の抵抗
#define MOTOR_EFFICIENCY (0.69)      //モーターの最大効率
#define MOTOR_FRICTION_TORQUE (0.00018)   //モーターの摩擦トルク[Nm]
#define GEAR_FRICTION_TORQUE  (0.00000)        //ギアの摩擦トルク
#define GROUND_FRICTION_TORQUE (0.00005)   //モーターの摩擦
/**********マシン固有定数(回路)***************/
#define V_REF        (3.278)          //テスターで計った3.3V系の電圧
#define OFFSET_V     (1.0)
#define OFFSET_ANG_V (1.0)
#define PROP_GYRO    (0.67)          //Gyroの比例係数
#define CENTER_R     (464)           //壁の中央での値
#define CENTER_L     (470)
/**********便利系定数******************/
//#define PI (3.1415926535897932384626433832795)
#define SQRT2 (1.4142356237)
#define LEN_BLOCK    (0.18)


#endif
