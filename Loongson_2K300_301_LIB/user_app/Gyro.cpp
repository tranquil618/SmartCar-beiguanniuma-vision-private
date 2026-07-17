#include "Gyro.hpp"
#include "lq_drv_inc.hpp"
#include "lq_app_inc.hpp"
#include "lq_common.hpp"
#include <chrono>
#include "Tof.hpp"
// 全局变量定义
eular_info eulerAngle = {0};
quarter_info Q_info = {1.0, 0.0, 0.0, 0.0};
gyroOffset_info gyroOffset = {0};
accOffset_info accOffset = {0};
double yaw_angle = 0.0;
int gyroflag = 0;

extern lq_udp_client syd_udp;

float last_raw_yaw = 0.0f;
float continuous_yaw = 0.0f;

//内部数据结构：滤波后的传感器数据


mpu_data_t  mpu_data = {0}; 
lq_i2c_mpu6050 mpu6050; 

// 快速平方根倒数,用于归一化
static inline float fast_invsqrt(float x) {
    float xhalf = 0.5f * x;
    int i = *(int*)&x;
    i = 0x5f3759df - (i >> 1);
    x = *(float*)&i;
    x = x * (1.5f - xhalf * x * x);
    return x;
}

//陀螺仪原始值转换为角速度rad/s
static inline float gyro_raw_to_radps(int16_t raw, float offset) {
    return ((float)(raw - offset) / Gyro_LSB_PER_DPS) * (PI / 180.0f);
}

// 加速度计原始值转换为 g
static inline float acc_raw_to_g(int16_t raw,float offset) {
    return (float)(raw - offset) / ACC_LSB_PER_G;
}


 // 陀螺仪零偏校准 
void gyroOffsetInit(void)
{
    int16_t gx, gy, gz;
    const int samples = 1000; // 校准样本数量
    double sum_gx = 0, sum_gy = 0, sum_gz = 0; //误差累计
    printf("陀螺仪正在校准\n");
    for (int i = 0; i < samples; i++) {
        if (mpu6050.get_mpu6050_ang(&gx, &gy, &gz)) {
            sum_gx += gx;
            sum_gy += gy;
            sum_gz += gz;
        } else {
            printf("读取失败\n");
            return;
        }
        usleep(1000);   // 5ms 间隔
    }

    gyroOffset.Xdata = (double)sum_gx / samples;
    gyroOffset.Ydata = (double)sum_gy / samples;
    gyroOffset.Zdata = (double)sum_gz / samples;

    gyroflag = 1; // 校准完成
    yaw_angle = 0.0;  //重置角度
    last_raw_yaw = 0.0f;
    continuous_yaw = 0.0f;
    printf("陀螺仪校准完毕， Bias: X=%.1f, Y=%.1f, Z=%.1f (LSB)\n",
           gyroOffset.Xdata, gyroOffset.Ydata, gyroOffset.Zdata);
}

//加速度计静态校准
void accelOffset_init(void) {
    int16_t ax, ay, az;
    double sum_ax = 0, sum_ay = 0, sum_az = 0;
    const int samples = 1000;

    printf("加速度计正在校准\n");
    for (int i = 0; i < samples; i++) {
        mpu6050.get_mpu6050_acc(&ax, &ay, &az);
        sum_ax += ax;
        sum_ay += ay;
        sum_az += az;
        usleep(1000);
    }
    float mean_ax = (float)sum_ax / samples;
    float mean_ay = (float)sum_ay / samples;
    float mean_az = (float)sum_az / samples;

    const float THEORETICAL_X = 0.0f;
    const float THEORETICAL_Y = 0.0f;
    const float THEORETICAL_Z = 8192.0f; 

    accOffset.Xdata = mean_ax - THEORETICAL_X;
    accOffset.Ydata = mean_ay - THEORETICAL_Y;
    accOffset.Zdata = mean_az - THEORETICAL_Z;
    printf("加速度计校准完毕， Bias: X=%.1f, Y=%.1f, Z=%.1f (LSB)\n",
           accOffset.Xdata, accOffset.Ydata, accOffset.Zdata);
}

//初始化陀螺仪，检查 ID，校准零偏
void gyro_init(void) {
    // 检查设备是否存在
    if (mpu6050.get_mpu6050_id() != 0x68) {
        printf("MPU6050 not found!\n");
        return;
    }
    
    gyroOffsetInit();
    accelOffset_init();
}

// 获取并滤波传感器数据，一阶低通滤波 

void  MPU_get_data(void){
    if (!gyroflag)  return;

    int16_t ax, ay, az, gx, gy, gz;
    if (!mpu6050.get_mpu6050_gyro(&ax, &ay, &az, &gx, &gy, &gz)) {
        printf("读取传感器数据失败\n");
        return;
    }

    //加速度进行滤波
    const float alpha = 0.3f; 
    float acc_x = acc_raw_to_g(ax, accOffset.Xdata);
    float acc_y = acc_raw_to_g(ay, accOffset.Ydata);
    float acc_z = acc_raw_to_g(az, accOffset.Zdata);
    mpu_data.ax = alpha * acc_x + (1.0f - alpha) * mpu_data.ax;
    mpu_data.ay = alpha * acc_y + (1.0f - alpha) * mpu_data.ay;
    mpu_data.az = alpha * acc_z + (1.0f - alpha) * mpu_data.az;

    // 角速度进行滤波
    // 1. 先进行原始数据转换，得到当前的粗糙角速度 (rad/s)
    float raw_gx = gyro_raw_to_radps(gx, gyroOffset.Xdata);
    float raw_gy = gyro_raw_to_radps(gy, gyroOffset.Ydata);
    float raw_gz = gyro_raw_to_radps(gz, gyroOffset.Zdata);

    // 2. 限幅滤波：干掉电磁干扰打出来的离谱错码
    // 设定阈值：10.0 rad/s (约等于 570度/秒)。三轮车不可能转这么快！
    // 如果超过这个阈值，说明是假数据，直接舍弃，沿用上一次的 mpu_data 值
    if (fabs(raw_gx) > 15.0f) raw_gx = mpu_data.gx;
    if (fabs(raw_gy) > 15.0f) raw_gy = mpu_data.gy;
    if (fabs(raw_gz) > 15.0f) raw_gz = mpu_data.gz;

    // 3. 一阶低通滤波：让曲线更加丝滑
    const float alpha_g = 0.6f; 
    mpu_data.gx = alpha_g * raw_gx + (1.0f - alpha_g) * mpu_data.gx;
    mpu_data.gy = alpha_g * raw_gy + (1.0f - alpha_g) * mpu_data.gy;
    mpu_data.gz = alpha_g * raw_gz + (1.0f - alpha_g) * mpu_data.gz;
}


//互补滤波更新四元数
void MPU_update(float gx, float gy, float gz, float ax, float ay, float az,float dt) {
    //float halfT = 0.5f * DELTA_T;
    float halfT = 0.5f * dt;
    float vx, vy, vz;   // 重力向量估计值
    float ex, ey, ez;   // 误差
    static float I_ex = 0, I_ey = 0, I_ez = 0;  // 误差积分

    // 加速度数据归一化
    float norm = fast_invsqrt(ax * ax + ay * ay + az * az);
    ax *= norm;
    ay *= norm;
    az *= norm;

    // 根据当前四元数计算重力向量
    float q0 = Q_info.q0, q1 = Q_info.q1, q2 = Q_info.q2, q3 = Q_info.q3;
    vx = 2.0f * (q1 * q3 - q0 * q2);
    vy = 2.0f * (q0 * q1 + q2 * q3);
    vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    // 叉积计算误差
    ex = ay * vz - az * vy;
    ey = az * vx - ax * vz;
    //ez = ax * vy - ay * vx;
    ez=0.0f;

    // 误差积分（比例 + 积分）
    I_ex += ex * halfT;
    I_ey += ey * halfT;
    I_ez += ez * halfT;

    // 使用 PI 修正陀螺仪测量值（Kp, Ki 可调）
    const float Kp = 0.5f;   // 比例增益
    const float Ki = 0.0001f;  // 积分增益
    gx += Kp * ex + Ki * I_ex;
    gy += Kp * ey + Ki * I_ey;
    gz += Kp * ez + Ki * I_ez;

    // 四元数微分方程（一阶龙格-库塔）
    q0 += (-q1 * gx - q2 * gy - q3 * gz) * halfT;
    q1 += ( q0 * gx + q2 * gz - q3 * gy) * halfT;
    q2 += ( q0 * gy - q1 * gz + q3 * gx) * halfT;
    q3 += ( q0 * gz + q1 * gy - q2 * gx) * halfT;

    // 归一化四元数
    norm = fast_invsqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    Q_info.q0 = q0 * norm;
    Q_info.q1 = q1 * norm;
    Q_info.q2 = q2 * norm;
    Q_info.q3 = q3 * norm;
}

//将四元数转换为欧拉角
void MPU_getEulerianAngles(void) {
    float q0 = Q_info.q0, q1 = Q_info.q1, q2 = Q_info.q2, q3 = Q_info.q3;
    // 俯仰角（Pitch）
    eulerAngle.pitch = -atan2f(2.0f * (q2 * q3 + q0 * q1), -2.0f * (q1 * q1 + q2 * q2) + 1.0f) * (180.0f / PI);
    // 横滚角（Roll）
    eulerAngle.roll  = asinf(2.0f * (q0 * q2 - q1 * q3)) * (180.0f / PI);
    // 偏航角（Yaw）
    //eulerAngle.yaw   = atan2f(2.0f * (q1 * q2 + q0 * q3), -2.0f * (q2 * q2 + q3 * q3) + 1.0f) * (180.0f / PI);



    // ========== 偏航角（Yaw）连续化处理 ==========
    // 1. 先算出原始的、带跳变的角度 (-180 到 180)
    /*float raw_yaw = atan2f(2.0f * (q1 * q2 + q0 * q3), -2.0f * (q2 * q2 + q3 * q3) + 1.0f) * (180.0f / PI);

    // 2. 计算本次角度和上次角度的差值
    float delta_yaw = raw_yaw - last_raw_yaw;

    // 3. 判断是否发生了跨界跳变，并进行补偿
    if (delta_yaw < -180.0f) {
        delta_yaw += 360.0f;   // 从 +179 跳到 -179 时，说明是逆时针转过界，差值会接近 -360，补回来
    } else if (delta_yaw > 180.0f) {
        delta_yaw -= 360.0f;   // 从 -179 跳到 +179 时，说明是顺时针转过界，差值会接近 +360，减回去
    }

    // 4. 将真实的增量累加到连续角度上
    continuous_yaw += delta_yaw;
    last_raw_yaw = raw_yaw; // 更新历史值供下次使用

    // 5. 最终输出给整个系统的 Yaw 角度
    eulerAngle.yaw = continuous_yaw;

    // 角度范围限制
    //if (eulerAngle.yaw < 0) eulerAngle.yaw += 360.0f;
    //if (eulerAngle.yaw >= 360) eulerAngle.yaw -= 360.0f;

   // 修改后的代码（只发 Yaw 和 Z轴角速度）：
    //char udp_buf[128];                                      
    // 注意：如果想看度/秒，可以将 mpu_data.gz 乘以 57.29578f
    //snprintf(udp_buf, sizeof(udp_buf), "%.2f,%.2f\n", (float)eulerAngle.yaw, (float)mpu_data.gz*57.29578f); // 发送偏航角和Z轴角速度（度/秒）
    //syd_udp.udp_send_string(udp_buf);*/
}

/* 单次姿态解算：读取数据、更新四元数、计算欧拉角 */
void MPU_loop(void) {

    static auto last_time = std::chrono::steady_clock::now();
    auto current_time = std::chrono::steady_clock::now();

    // 计算两次调用之间的真实时间差（单位：秒）
    float dt = std::chrono::duration<float>(current_time - last_time).count();
    last_time = current_time;

    if (dt <= 0.0f || dt > 0.05f) {
        dt = 0.005f; 
    }

    MPU_get_data();                 // 获取并滤波数据

    MPU_update(mpu_data.gx, mpu_data.gy, mpu_data.gz,
             mpu_data.ax, mpu_data.ay, mpu_data.az,dt);
    MPU_getEulerianAngles();        // 更新欧拉角
    // 🌟 2. 智能车专用的降维打击：2D Yaw 纯积分 🌟
    // 直接把 rad/s 乘以时间 dt，再转成角度，累加到全局变量 yaw_angle 上
    yaw_angle += mpu_data.gz * dt * (180.0f / PI); 

    // 2. 将角度严格约束在 -360 到 +360 之间
    while (yaw_angle >= 360.0f) {
        yaw_angle -= 360.0f;
    }
    while (yaw_angle <= -360.0f) {
        yaw_angle += 360.0f;
    }
    
    // 强行覆盖掉四元数算出来的那个严重失真的 Yaw！
    eulerAngle.yaw = yaw_angle;
}

void gyro_timer_callback() {
    MPU_loop();
    tof_get_data();
}
