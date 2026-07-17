#include "Tof.hpp"
#include "lq_i2c_vl53l0x.hpp" // 引用龙邱的 VL53L0X 驱动
#include <chrono>
#include <iostream>
using namespace std;

// 全局变量定义
tof_data_t tof_data = {8192.0f, 8192.0f, 0.0f};


float DL1A_Dis = 8192.0f;
float DL1A_Dis_Diff = 0.0f;

// 像 MPU6050 一样，直接在全局静态实例化设备对象
lq_i2c_vl53l0x vl53l0x;

// 卡尔曼滤波器内部状态变量
static float KF_Q = 0.25f; // 过程噪声协方差（越小越相信上一次预测）
static float KF_R = 2.5f;  // 测量噪声协方差（越大滤波越平滑）
static float KF_P = 1.0f;  // 估计误差协方差
static float KF_x = 8192.0f; // 最优估计值（初始设为无穷远）

void tof_init(void) {
    // 对象的构造函数已在全局执行，这里初始化变量即可
    KF_x = 8192.0f;
    tof_data.filter_dis = 8192.0f;
    tof_data.diff_dis = 0.0f;
    printf("VL53L0X TOF Sensor Initialized.\n");
}

/* 单次获取测距数据、进行滤波、计算微分 */
void tof_get_data(void) {
    // 1. 获取原始距离
    uint16_t dis = vl53l0x.get_vl53l0x_dis();
    
    // 如果测距仪超出量程或未收到反馈，返回极大值 (8192代表安全/无障碍)
    if (dis == 0 || dis > 2000) { 
        dis = 8192;
    }
    tof_data.raw_dis = (float)dis;

    // 2. 一阶卡尔曼滤波 (如果收到的是 8192 越界值，直接赋值不滤波，防止拖尾)
    if (dis == 8192) {
        KF_x = 8192.0f;
    } else {
        KF_P = KF_P + KF_Q;
        float K = KF_P / (KF_P + KF_R); // 计算卡尔曼增益
        KF_x = KF_x + K * (tof_data.raw_dis - KF_x); // 更新最优估计值
        KF_P = (1.0f - K) * KF_P;
    }

    // 3. 计算精确的时间差 dt (模仿 Gyro.cpp 里的 chrono 做法)
    static auto last_time = std::chrono::steady_clock::now();
    auto current_time = std::chrono::steady_clock::now();
    float dt = std::chrono::duration<float>(current_time - last_time).count();
    last_time = current_time;

    // 时间差容错处理 (限制在 10ms - 200ms 之间，避免开机第一次计算出极大值)
    if (dt <= 0.01f || dt > 0.2f) {
        dt = 0.05f; 
    }

    // 4. 计算微分 (速度) 与 更新数据结构
    tof_data.diff_dis = (KF_x - tof_data.filter_dis) / dt;
    tof_data.filter_dis = KF_x;

    // 5. 同步更新给 Ramp 使用的全局变量
    DL1A_Dis = tof_data.filter_dis;
    DL1A_Dis_Diff = tof_data.diff_dis;
    //cout<<"DL1A_Dis: "<<DL1A_Dis<<" mm, DL1A_Dis_Diff: "<<DL1A_Dis_Diff<<" mm/s"<<endl; // 实时打印距离和速度，方便调试
}

