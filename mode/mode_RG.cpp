



#include <mouse.h>
#include <myUtil.h>
#include <pathCalculation.h>
#include <stdint.h>
#include <uart.h>
#include <wallsensor.h>
#include "mode_GB.h"
#include "tactsw.h"
#include "timer.h"
#include "fcled.h"
#include "sound.h"
#include "pwm.h"
#include "gamepad.h"

#include "sound.h"
#include "ICM20602.h"
#include "moveEvent.h"
#include "communication.h"
#include "parameterManager.h"
#include <vector>

using std::vector;

using namespace robot_object;
using namespace umouse_object;


void mode_RG(){
    printfAsync("RG mode\n");
    printfAsync("迷路データ消去\n");

    UMouse& m = UMouse::getInstance();
    m.maze.init();
    SEC();
    waitmsec(200);
    SEC();
    waitmsec(200);
    SEC();
    waitmsec(200);
};



