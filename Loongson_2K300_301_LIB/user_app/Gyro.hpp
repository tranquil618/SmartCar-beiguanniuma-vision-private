#ifndef GYRO_HPP
#define GYRO_HPP
#include "lq_drv_inc.hpp"
#include "lq_app_inc.hpp"
#include "lq_common.hpp"

#define Gyro_LSB_PER_DPS 16.4f  // 陀螺仪灵敏度：16.4 LSB/(°/s)
#define ACC_LSB_PER_G 8192.0f   // 加速度计灵敏度：8192 LSB/g
#define DELTA_T 0.0025f         // 采样时间间隔：2.5ms
#define PI      3.14159265358979323846f

typedef struct {
    double pitch;//俯仰角
    double roll;//翻滚角
    double yaw;//偏航角
} eular_info;  // 欧拉角结构体

typedef struct {
    double q0;
    double q1;
    double q2;
    double q3;
} quarter_info;  // 四元数结构体

typedef struct {
    double Xdata;
    double Ydata;
    double Zdata;
} gyroOffset_info;  // 陀螺仪偏移量结构体

typedef struct {
    double Xdata;
    double Ydata;
    double Zdata;
} accOffset_info;  // 加速度计偏移量结构体

typedef struct {
    float ax,ay,az;  // 加速度（g）
    float gx,gy,gz; // 角速度（rad/s）
} mpu_data_t;// 滤波后的传感器数据结构体

extern quarter_info Q_info;  // 四元数（保留）
extern eular_info eulerAngle;   // 欧拉角（度）
extern gyroOffset_info gyroOffset;   // 陀螺仪零偏（原始 LSB 值）
extern accOffset_info accOffset;     // 加速度计零偏（保留）
extern mpu_data_t mpu_data; // 滤波后的传感器数据（保留）
extern double yaw_angle;                    // 偏航角（度），与逐飞库命名一致
extern int gyroflag;                        // 零漂校准完成标志

void gyroOffsetInit(void); // 陀螺仪零偏校准
void accelOffset_init(void) ; // 加速度计零偏校准

void gyro_init(void);
void MPU_get_data(void); // 获取并滤波传感器数据
void MPU_update(float gx, float gy, float gz, float ax, float ay, float az); // 更新四元数和欧拉角
void MPU_loop(void); // 单次姿态解算：读取数据、更新四元数、计算欧拉角
void gyro_timer_callback() ;
#endif // GYRO_HPP