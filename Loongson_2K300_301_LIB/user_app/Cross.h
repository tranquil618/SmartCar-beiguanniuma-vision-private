/*
*Cross.h
*
*Created on: 2026年4月3号
*  Author：LYC
*/
#ifndef CODE_CROSS_H_
#define CODE_CROSS_H_
#include "Image.h"

extern float FarLeft_Line[MAX_POINTS][2];
extern int Far_Left_Num;
extern float FarRight_Line[MAX_POINTS][2];
extern int Far_Right_Num;

extern float FarBlueLeft_Line[MAX_POINTS][2];
extern float FarBlueRight_Line[MAX_POINTS][2];

extern float FarRepLeft_Line[MAX_POINTS][2];
extern int FarRepLnum;
extern float FarRepRight_Line[MAX_POINTS][2];
extern int FarRepRnum;

extern float FarLeftAngel[MAX_POINTS];
extern float FarRightAngel[MAX_POINTS];

extern float FarNmsLeftAngel[MAX_POINTS];
extern float FarNmsRightAngel[MAX_POINTS];
extern LP_t FarLeftP,FarRightP;
extern uint16 CrossEnc_TH;
extern float Cross_Dis;
extern uint8 Out_CrossTH;
extern int start_test;

typedef enum{
    Cross_Left=0,
    Cross_Right,
    Cross_ALL,
    Cross_IN,
    Cross_NULL=0xff
}Cross_t;

extern Cross_t CrossType;
void Cross_Check(void);
void Cross_Loop(void);
float Cross_MidLine_Ctrl(void);
uint8 Cross_Refind_Line(void);
#endif /*CODE_CROSS_H_*/