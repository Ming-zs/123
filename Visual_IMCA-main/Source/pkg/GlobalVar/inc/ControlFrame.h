#ifndef CONTROL_FRAME__
#define CONTROL_FRAME__

#include <stdint.h>
#include "Serial/FoundationType/inc/FLOAT32.h"

#define MVCAMERA_FX 1405.9
#define MVCAMERA_FY 1405.2
#define MVCAMERA_CX 539.5263 // 小往you
#define MVCAMERA_CY 475.2161 // da往高

struct ControlFrame{
    FLOAT32   yaw;
    FLOAT32   pitch;
    uint8_t seq = 0;
    uint8_t model = 0;
};

#endif