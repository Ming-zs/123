//
// Created by experi on 2023/3/4.
//

#ifndef SSI_H
#define SSI_H
#include <cstdint>
#include "FLOAT32.h"

struct SSI{
    uint8_t SOF = 0;                // 帧头
    uint8_t robotModel = 0;         //  机器人模式
    uint8_t shutdownPC = 0;         // PC状态
    uint8_t robotColor = 0;         //  机器人颜色
    uint8_t CRC8 = 0;

    uint8_t ssType = 's';           // 机器人职业
    uint8_t bulletSpeed = 10;       // 子弹速度

    FLOAT32 currentYaw;         // 当前yaw
    FLOAT32 currentPitch;       // 当前Pitch
    FLOAT32_2 robotForward;      // 当前运动方向， -1表示未运动
};

#endif // ROOT_IMCA_SSI_H
