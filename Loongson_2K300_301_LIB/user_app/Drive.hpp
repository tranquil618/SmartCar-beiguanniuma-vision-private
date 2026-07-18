#ifndef DRIVE_HPP
#define DRIVE_HPP

#include "PID.hpp"
#include <cstdint>

typedef struct Motor
{
    int  duty;
    int Target_Speed;
    float Encoder_Speed;
} Motor;

extern float DL1A_Dis;
extern float DL1A_Dis_Diff;

extern Motor Motor_L; //左轮
extern Motor Motor_R; //右轮

extern float K_turn; // 转向系数


void Init();
void Right_Motor_Set_Speed(int pwm) ;
void Left_Motor_Set_Speed(int pwm) ;
void Motor_Control(int Speed_L, int Speed_R); // 电机控制函数
void Set_Fan_Speed(float throttle_1, float throttle_2) ;

//获取编码器速度
void Encoder_Speed_Get(float *Speed_L, float *Speed_R);
int64_t Get_IENC(void);



void kaihuan() ;

// 获取编码器速度（声明一次）
void Encoder_Speed_Get(float *Speed_L, float *Speed_R);

void Control_SetBaseSpeed(float speed);

void Control_UpdateCameraError(float error);

void angle_control_callback(float error);
void omega_control_callback();
void speed_control_callback(float error);
void angle_control_thread();
void omega_control_thread();
void speed_control_thread();

void control_callback();
void UDP_data(void);
void UDP_data_VOFA1(void);

// 设置基础速度（Drive.cpp 中实现）
void Set_Base_Speed(float speed);

void Set_Target_Speed(float speed);
// 使用 <cstdint> 中的标准类型
int64_t Get_IENC(void);

#endif