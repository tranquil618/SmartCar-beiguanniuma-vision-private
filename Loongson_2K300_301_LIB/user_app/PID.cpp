#include <iostream>
#include "PID.hpp"
#include "main.hpp"
#include <algorithm>
using namespace std;

// 实例定义（在一个翻译单元中定义 extern 声明的 PID 结构体）
SpeedPID Motor_PID_L;
SpeedPID Motor_PID_R;
AnglePID Angle_PID;
OmegaPID Omega_PID;

void AnglePID_Init(AnglePID *pid, float Kp, float Kd, float maxOut) {
    pid->Kp = Kp;  pid->Kd = Kd;
    pid->Error = 0; pid->Last_Error = 0;
    pid->Output = 0; pid->Last_Output = 0;
    pid->MaxOut = maxOut;
}
void OmegaPID_Init(OmegaPID *pid, float Kp,float Kd, float maxOut) {
    pid->Kp = Kp;pid->Kd = Kd;
    pid->Error = 0; pid->Last_Error = 0;
    pid->Output = 0; pid->Last_Output = 0;
    pid->MaxOut = maxOut;
}
void SpeedPID_Init(SpeedPID *pid, float Kp, float Ki, float maxOut) {
    pid->Kp = Kp; pid->Ki = Ki;
    pid->Error = 0; pid->Last_Error = 0;
    pid->Output = 0; pid->Last_Output = 0;
    pid->MaxOut = maxOut;
}

//左偏：measure小于0，error大于0，output是正的，差速加到右轮
//右偏：measure大于0，error小于0，output是负的，差速加到左轮

// ========== 角度环 ==========
float AnglePID_Calc(AnglePID *pid, float measure) {
    // 更新误差
    pid->Last_Error = pid->Error;
    pid->Error = measure;

    //cout<<"角度误差: "<<pid->Error<<endl;
    //float abs_err = fabs(pid->Error);
    //float dynamic_Kp = pid->Kp + (abs_err * 0.15f);

   // if (dynamic_Kp > 7.0f) {
       // dynamic_Kp = 7.0f;
    //}
    float P = pid->Kp * pid->Error;
    //float P = dynamic_Kp * pid->Error;
    float D = pid->Kd * (pid->Error - pid->Last_Error);
    
    // 输出期望角速度 Target_Omega
    float raw_output = P + D;
 
    if (raw_output > pid->MaxOut) raw_output = pid->MaxOut;
    if (raw_output < -pid->MaxOut) raw_output = -pid->MaxOut;
    //cout<<"角度环输出（差速量）: "<<pid->Output<<endl;

    pid->Output = raw_output;

    pid->Last_Output = pid->Output;
    return pid->Output;
}

// ========== 角速度环 ==========

float OmegaPID_Calc(OmegaPID *pid, float target_omega, float gyro_z) {
    pid->Last_Error = pid->Error;

    pid->Error = target_omega - gyro_z;
    //cout<<"角速度误差: "<<pid->Error<<endl;

    float D = pid->Kd * (pid->Error - pid->Last_Error);
    float P = pid->Kp * pid->Error;
    
    float raw_output = P + D;

    if (raw_output > pid->MaxOut) raw_output = pid->MaxOut;
    if (raw_output < -pid->MaxOut) raw_output = -pid->MaxOut;
    //cout<<"角速度环输出（差速量）: "<<pid->Output<<endl;
    // 🌟 核心杀手锏：一阶低通滤波（保护齿轮，消除卡卡声）
    // A 的值在 0.0 到 1.0 之间。A 越大，缓冲越强，电机越柔和。
    // 建议设为 0.6f 到 0.8f 之间。
    float A = 0.2f; 
    pid->Output = A * pid->Last_Output + (1.0f - A) * raw_output;

    pid->Last_Output = pid->Output;
    return pid->Output;
}

// ===================== 左轮速度环 =====================
float Left_SpeedPID(SpeedPID *pid, float target_speed, float now_speed) {
    float A = 0.85f;   // 低通滤波系数，可按需调整
    // 计算期望速度

    // 保存上一次输出（用于低通滤波）
    pid->Last_Output = pid->Output;
    pid->Last_Error = pid->Error;
    //cout<<"左误差: "<<pid->Error<<endl;

    pid->Error = target_speed - now_speed;
    //cout<<"左速度误差: "<<pid->Error<<endl;
    
    float P = (pid->Kp+5) * (pid->Error - pid->Last_Error);
    float I = (pid->Ki+0.04) * pid->Error;
    float output = P + I;
    
    // 累加输出
    pid->Output += output;
    // 输出限幅
    if (pid->Output > pid->MaxOut) pid->Output = pid->MaxOut;
    if (pid->Output < -pid->MaxOut) pid->Output = -pid->MaxOut;
    
    // 低通滤波
    pid->Output = A * pid->Output + (1 - A) * pid->Last_Output;
    return pid->Output;
}

// ===================== 右轮速度环  =====================
float  Right_SpeedPID(SpeedPID *pid, float target_speed, float now_speed) {
    float A = 0.85f;
    // 计算期望速度（右轮减差速量）
    
    pid->Last_Output = pid->Output;
    
    pid->Last_Error = pid->Error;
    pid->Error = target_speed - now_speed;
    // cout<<"右误差: "<<pid->Error<<endl;
    
    float P = pid->Kp * (pid->Error - pid->Last_Error);
    float I = (pid->Ki -0.04) * pid->Error;
    float output = P + I;
    
    pid->Output += output;
    // 输出限幅
    if (pid->Output > pid->MaxOut) pid->Output = pid->MaxOut;
    if (pid->Output < -pid->MaxOut) pid->Output = -pid->MaxOut;
    
    pid->Output = A * pid->Output + (1 - A) * pid->Last_Output;
    return pid->Output; 
}


