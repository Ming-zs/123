// create by experi	 2023-03-04 10:30:12

#ifndef GLOBALVAR__H__
#define GLOBALVAR__H__

#include "Serial/FoundationType/inc/SSI.h"
#include <mutex>

extern SSI g_ssi;                   // 从这里获取自身信息
extern std::mutex mt_g_ssi;         // 获取自身信息的时候要加锁

#endif
