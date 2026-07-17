#include "Ramp.hpp"
#include "Drive.hpp"   // 包含 Set_Target_Speed, Get_IENC
#include "Gyro.hpp"    // 包含 eulerAngle
#include "Image.h"     // 包含图像数据 Image_Used, OSTU_threshold, LD, RD 等
#include "Tof.hpp"     // 包含 DL1A_Dis, DL1A_Dis_Diff
#include "Rot.h"       // 包含逆透视数组 inv_mapx, inv_mapy
#include <cmath>
#include <algorithm>
#include <iostream>
using namespace std;

Ramp_t Ramp_type = Ramp_NULL;
float RampMID[100][2];
int RampMIDNum = 0;
extern int Rep_Lnum; // 引入左边线有效点数
extern int Rep_Rnum; // 引入右边线有效点数

static float start_pitch = 0;
uint8_t Check_Num = 0;
float LD = 1000.0f;
float RD = 1000.0f;
int state_frame_cnt = 0;


// === 硬件阈值参数 (需根据龙邱车模实测调整) ===
uint16_t TOF_RAMP_TH = 200;       // 距离坡道多远时认为要上坡 (mm)
uint16_t TOF_DOWN_TH = 200;       // 下坡时测距仪测到的距离 (mm)
float Up_Down_Pitch_TH = 4.5f;    // 陀螺仪仰角变化阈值 (度)
float Ramp_Out_Pitch_TH = 2.5f;   // 认为坡道结束的仰角容差 (度)
// ==============================================

static int64_t Speed_I = 0; // 用于记录进入各阶段时的编码器里程积分

// 坡道专属中线生成 (简单取左右边线的平均)
static inline void Mid_Ramp(void) {
    int num = std::min(Rep_Lnum, Rep_Rnum);
    for(int i = 0; i < std::min(num, 100); ++i) {
        RampMID[i][0] = (Rep_Left_Line[i][0] + Rep_Right_Line[i][0]) / 2.0f;
        RampMID[i][1] = (Rep_Left_Line[i][1] + Rep_Right_Line[i][1]) / 2.0f;
    }
    RampMIDNum = std::min(num, 100);
}

// 计算某一行(Line)赛道的物理宽度和偏角
// 计算某一行(Line)赛道的物理宽度和偏角
static inline float Track_Wide(uint8_t Line, float* K) {
    int L = 1, R = IMG_W - 2; // 【修改点】将 uint8_t 改为 int，防止 320 越界
    
    // 从中间向右扫右边缘
    for(int i = IMG_W >> 1; i < IMG_W - 1; ++i) { // 【修改点】循环变量 i 也改为 int
        if(AT(i,Line) > OSTU_threshold && AT(i+1,Line) <= OSTU_threshold) {
            R = i; break;
        }
    }
    // 从中间向左扫左边缘
    for(int i = IMG_W >> 1; i > 1; --i) { // 【修改点】循环变量 i 改为 int
        if(AT(i,Line) > OSTU_threshold && AT(i-1,Line) <= OSTU_threshold) {
            L = i; break;
        }
    }
    // 转物理坐标
    float xr = inv_mapx[Line][R];
    float yr = inv_mapy[Line][R];
    float xl = inv_mapx[Line][L];
    float yl = inv_mapy[Line][L];
    
    // 计算偏角 (度)
    *K = fabs(atan2f(yr - yl, xr - xl)) / PI * 180.0f;
    return (xr - xl); // 返回物理宽度
}

// 坡道识别函数 (核心预警)
void Ramp_Check(void) {
#define Check_TH (15) // 需要满足梯形特征的行数阈值
    RampMIDNum = 0;
    //cout<<" DL1A_Dis: "<<DL1A_Dis<<" mm, DL1A_Dis_Diff: "<<DL1A_Dis_Diff<<" mm/s"<<endl; // 实时打印距离和速度，方便调试
    //cout<<"pitch: "<<eulerAngle.pitch<<endl;
    if(Ramp_type == Ramp_NULL) {
        // 1. TOF 预警：距离突减，或距离小于阈值的 1.25 倍
        if(DL1A_Dis <= TOF_RAMP_TH ) {
            
            // 2. 图像直道预警：左右边线平直 (LD/RD极小)，且前瞻足够远
            // 注：240行图像，Long_WhiteLine_Y 是从上往下的坐标，所以越小代表前瞻越远
            int white_height = IMG_H - Long_WhiteLine_Y; 
            //cout<<"LD: "<<LD<<"  RD: "<<RD<<"  white_height: "<<white_height<<endl; // 实时打印边距和前瞻高度，方便调试
            
            float avg_LD = (Rep_Lnum > 0) ? (LD / Rep_Lnum) : 1000.0f;
            float avg_RD = (Rep_Rnum > 0) ? (RD / Rep_Rnum) : 1000.0f;
            //cout << "avg_LD: " << avg_LD << " | avg_RD: " << avg_RD<< "  white_height: "<<white_height<< endl;
            
            if(LD > 0 && RD > 0 && avg_LD < 3.5f && avg_RD < 3.5f && white_height >= 120) {
                
                uint8_t check_num = 0;
                // 3. 图像梯形特征确认：赛道是否呈现“上坡变宽”的透视畸变
                // 原代码扫 65~25，适配 240 行图像，放大扫描范围：130~50
                for(uint8_t i = 130; i > 50; --i) {
                    float K = 0, K1 = 0;
                    float wide_current = Track_Wide(i, &K);
                    float wide_prev = Track_Wide(i - 2, &K1); // 隔行采样，差值更明显
                    
                    // 如果赛道宽度在变宽，且物理宽度大于 50mm
                    if(wide_prev - wide_current > 0 && wide_current > 50) {
                        // 且边缘没有剧烈弯曲
                        if(K < 1.5f && K1 < 1.5f) check_num++;
                    }
                }
                
                // 最终起坡确认
                if(check_num > Check_TH && DL1A_Dis <= TOF_RAMP_TH) {
                    Check_Num = check_num;
                    Ramp_type = Ramp_Font;
                    start_pitch = eulerAngle.pitch; // 记录平地仰角
                    state_frame_cnt=0;          // 记录当前里程积分
                }
            }
        }
    }
}

// 坡道状态机循环
void Ramp_Loop(void) {
    Ramp_Check();
    //cout<<"Ramp_type: "<<Ramp_type<<endl; // 实时打印当前坡道状态和帧计数，方便调试
    if (Ramp_type != Ramp_NULL) {
        state_frame_cnt++; 
    }
    
    switch(Ramp_type) {
        case Ramp_Font:
            Set_Target_Speed(40); 
            Mid_Ramp();
            LRType = MID_Ramp;
            
            if(eulerAngle.pitch < -5.0f) {
                Ramp_type = Ramp_UP;
                state_frame_cnt = 0; 
            }
            // 【核心修改2：超时自愈！】
            // 如果平地颠簸真的让你误入状态1，但30帧(约0.3秒)内车头都没抬起来，说明是误判，强制退出！
            else if (state_frame_cnt > 30) {
                Ramp_type = Ramp_NULL;
                state_frame_cnt = 0;
            }
            break;
            
        case Ramp_UP:
            Set_Target_Speed(45); // 稳住上坡
            Mid_Ramp();
            LRType = MID_Ramp;
            
            if(eulerAngle.pitch > -3.0f && state_frame_cnt > 20) {
                Ramp_type = Ramp_Mid;
                state_frame_cnt = 0;
            }
            // 【兜底防卡死】：上坡跑了太久 (100帧)，强制切坡顶
            else if (state_frame_cnt > 100) {
                Ramp_type = Ramp_Mid;
                state_frame_cnt = 0;
            }
            break;
            
        case Ramp_Mid:
            Mid_Ramp();
            LRType = MID_Ramp;
            
            if((eulerAngle.pitch > 5.0f || DL1A_Dis > 400.0f) && state_frame_cnt > 15) {
                Ramp_type = Ramp_Down;
                state_frame_cnt = 0;
            }
            // 【兜底防卡死】：坡顶跑了太久 (80帧)，强制切下坡
            else if (state_frame_cnt > 80) {
                Ramp_type = Ramp_Down;
                state_frame_cnt = 0;
            }
            break;
            
        case Ramp_Down:
            Mid_Ramp();
            LRType = MID_Ramp;
            Set_Target_Speed(30); 
            
            if(eulerAngle.pitch < 3.0f && state_frame_cnt > 20) {
                Ramp_type = Ramp_Out;
                state_frame_cnt = 0;
            }
            // 【兜底防卡死】：下坡跑了太久 (100帧)，强制切出坡
            else if (state_frame_cnt > 100) {
                Ramp_type = Ramp_Out;
                state_frame_cnt = 0;
            }
            break;
            
        case Ramp_Out:
            Mid_Ramp();
            LRType = MID_Ramp;
            
            // 出坡后强制冷却 30 帧，这期间屏蔽一切平地颠簸导致的误判！
            if(state_frame_cnt > 30) {
                extern float base_speed;
                Set_Target_Speed(base_speed); 
                Ramp_type = Ramp_NULL; 
                state_frame_cnt = 0;
            }
            break;
            
        default:
            break;
    }
}