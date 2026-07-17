#ifndef CODE_CIRCLE_H_
#define CODE_CIRCLE_H
#include "Image.h"

typedef enum{
    Circle_LIN=0,   //入环阶段
    Circle_RIN,
    Circle_L,       //环内阶段
    Circle_R,
    Circle_LOut,    //出环阶段
    Circle_ROut,
    Circle_NULL=255 //正常/空闲状态，不触发任何圆环逻辑
}Circle_t;
extern Circle_t circle_type;
extern float Circle_IN_yaw;
extern float Circle_Out_yaw;
extern uint8 Circle_fps_th;
void Circle_Check(void);
void Circle_Loop(void);

#endif  //#ifndef CODE_CIRCLE_H