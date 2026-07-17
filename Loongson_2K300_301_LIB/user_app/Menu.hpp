#ifndef Menu_HPP
#define Menu_HPP

#include <stdint.h>


extern float kp_increase;
extern float kp_decrease;
extern int16_t Speed_Goal;
extern float limit_k;

extern float kp_increase_cir;
extern float kp_decrease_cir;
extern int16_t Speed_Goal_cir;
extern int16_t GOGOGO;
extern float limit_k_cir;

extern int16_t bao;   // 曝光时间参数

// 菜单入口函数
int menu1(void);                // 主菜单
int menu2_Speed_PID(void);      // 电机PID菜单
int menu2_Angle_PID(void);
void Load_All_Params(void);        // 从文件加载所有参数到全局变量


#endif // Menu_HPP