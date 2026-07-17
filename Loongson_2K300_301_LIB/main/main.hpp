#ifndef __MAIN_HPP
#define __MAIN_HPP

#include <stdio.h>

// 包含所有底层驱动头文件
#include "lq_drv_inc.hpp"

// 包含所有工具头文件
#include "lq_common.hpp"

// 包含所有应用层头文件
#include "lq_app_inc.hpp"

// 包含所有测试程序头文件
#include "lq_all_demo.hpp"

#include "PID.hpp"
#include "Gyro.hpp"
#include "Image.h"
#include "Cross.h"
#include "Circle.h"
//#include "Recognition.hpp"
#include "Rot.h"
#include "Drive.hpp"
#include "Ramp.hpp"
#include "Menu.hpp"
#include "Key.hpp"
#include <atomic>

// 全局共享变量声明
extern std::atomic<float> camera_error;
extern float base_speed;
//extern Image_Type TargetType; // 识别结果枚举值

// 确保主程序能调用菜单模块提供的配置加载函数
void Load_All_Params(void);
#endif
