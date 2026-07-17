#ifndef Tof_HPP
#define Tof_HPP

#include "lq_drv_inc.hpp"
#include "lq_app_inc.hpp"
#include "lq_common.hpp"

// 定义测距仪数据结构体 (模仿 mpu_data_t)
typedef struct {
    float raw_dis;      // 原始测量距离 (mm)
    float filter_dis;   // 滤波后的平滑距离 (mm)
    float diff_dis;     // 距离变化率 (微分)
} tof_data_t;

extern tof_data_t tof_data;


extern float DL1A_Dis; //滤波后的距离
extern float DL1A_Dis_Diff; //距离变化率

void tof_init(void);
void tof_get_data(void);        // 获取并滤波数据 (模仿 MPU_get_data)

#endif // TOF_HPP