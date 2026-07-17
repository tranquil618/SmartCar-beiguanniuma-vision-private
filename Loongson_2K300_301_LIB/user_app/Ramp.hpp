#ifndef RAMP_HPP
#define RAMP_HPP

#include <cstdint>

typedef enum {
    Ramp_NULL = 0,
    Ramp_Font,   // 坡道前沿（起步）
    Ramp_UP,     // 上坡中
    Ramp_Mid,    // 坡顶
    Ramp_Down,   // 下坡中
    Ramp_Out     // 出坡
} Ramp_t;

extern Ramp_t Ramp_type;
extern float RampMID[100][2]; 
extern int RampMIDNum;
extern uint8_t Check_Num; // 用于调试观察的特征点数量

extern float Mid_conf;
extern float LD,RD;  //左边线和右边线的距离，单位为像素
//extern float RampMID[70][2];  //环岛中线坐标数组 [点索引][0=X坐标,1=Y坐标]
//extern int RampMIDNum; //环岛中线坐标数组中的有效点数量
extern uint8_t Ramp_Basetime;
extern uint8_t Ramp_time;
extern float Up_Down_TH;
extern float Ramp_Out_TH;
extern uint16_t DL1A_TH;
extern uint16_t DL1A_Down_TH;

void Ramp_Check(void);
void Ramp_Loop(void);

#endif // RAMP_HPP