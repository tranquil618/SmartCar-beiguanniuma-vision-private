#include "lq_display_tft18.hpp"
#include "lq_reg_gpio.hpp"
#include "File_IO.hpp"
#include "Key.hpp"
#include "PID.hpp"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

float kp_increase = 0;
float kp_decrease = 0;
int16_t Speed_Goal = 0;
float limit_k = 0;

//文件读写宏
#define file_read_string  ::file_read_string
#define file_write_string ::file_write_string



//显示辅助函数
// 在指定像素坐标显示字符串（使用8x16字体）
static void menu_show_string(uint8_t x_pixel, uint8_t y_pixel, const char *str) {
    uint8_t char_x = x_pixel / 8;
    uint8_t char_y = y_pixel / 16;
    lq_tft18_drv_p8x16_str(char_x, char_y, str, U16WHITE, U16BLACK);
    lq_tft18_drv_flush();
}

// 在指定像素坐标显示整数
static void menu_show_int(uint8_t x_pixel, uint8_t y_pixel, int32_t value, uint8_t digits) {
    char buf[16];
    snprintf(buf, sizeof(buf), "%*d", digits, value);
    menu_show_string(x_pixel, y_pixel, buf);
}

// 在指定像素坐标显示浮点数
static void menu_show_float(uint8_t x_pixel, uint8_t y_pixel, float value, uint8_t int_digits, uint8_t frac_digits) {
    char buf[32];
    snprintf(buf, sizeof(buf), "%*.*f", int_digits + frac_digits + 1, frac_digits, value);
    menu_show_string(x_pixel, y_pixel, buf);
}

// 清屏
static void menu_clear(void) {
    lq_tft18_drv_cls(U16BLACK);
    lq_tft18_drv_flush();
}

// 按键获取
static int8_t Key_GetNum(void) {
    return key_get_num();  // 返回 1:上, 2:下, 3:短按(减), 4:长按(加)
}


///主菜单

// ---------------------------- 菜单实现 ----------------------------

int menu1(void) {
    uint8_t flag = 1;
    menu_clear();
    menu_show_string(0, 0,   "Speed_PID       ");
    menu_show_string(0, 16,  "Angle_PID       ");//角度环
    menu_show_string(0, 32,  "Reset           ");//重启
    menu_show_string(0, 48,  "Yes_Quit        ");//保存参数，退出菜单线程，开始跑车
    menu_show_string(0, 64,  "Image        ");   //退出这个线程，开启图像模式
    menu_show_string(0, 80,  "Baoguang        "); //调节曝光


    while (1) {
        int8_t KeyNum = Key_GetNum();
        if (KeyNum == 1) { // 上
            flag--;
            if (flag == 0) flag = 7;
        }
        if (KeyNum == 2) { // 下
            flag++;
            if (flag == 8) flag = 1;
        }
        if (KeyNum == 4) { // 长按确认/退出
            menu_clear();
            return flag;
        }

        // 清除所有指示箭头
        menu_show_string(96, 0,   "    ");
        menu_show_string(96, 16,  "    ");
        menu_show_string(96, 32,  "    ");
        menu_show_string(96, 48,  "    ");
        menu_show_string(96, 64,  "    ");
        menu_show_string(96, 80,  "    ");
        menu_show_string(96, 96,  "    ");

        // 显示当前选中箭头
        switch (flag) {
            case 1: menu_show_string(96, 0,   "<---"); break;
            case 2: menu_show_string(96, 16,  "<---"); break;
            case 3: menu_show_string(96, 32,  "<---"); break;
            case 4: menu_show_string(96, 48,  "<---"); break;
            case 5: menu_show_string(96, 64,  "<---"); break;
            case 6: menu_show_string(96, 80,  "<---"); break;
            case 7: menu_show_string(96, 96,  "<---"); break;
        }
    }
}

int menu2_Speed_PID(){
   uint8_t moflag = 1; 
   menu_clear();
   char Motor_L_Kp1[50] = {0};  //左电机Kp
   char Motor_L_Ki1[50] = {0};  //左电机Ki
   char Motor_R_Kp1[50] = {0};  //右电机Kp
   char Motor_R_Ki1[50] = {0};  //右电机Ki
   char Motor_kp_increase[50] = {0};  //速度增量
   char Motor_kp_decrease[50] = {0};  //速度减量
   char Motor_Speed[50] = {0};  //目标速度
   char Limit_k[50] = {0};  //限幅系数


    // 各个参数对应的文本文件路径
    const char *Motor_L_Kp1_txt = "Motor_L_Kp1.txt";
    const char *Motor_L_Ki1_txt = "Motor_L_Ki1.txt";
    const char *Motor_R_Kp1_txt = "Motor_R_Kp1.txt";
    const char *Motor_R_Ki1_txt = "Motor_R_Ki1.txt";
    const char *Motor_kp_increase_txt = "Motor_kp_increase.txt";
    const char *Motor_kp_decrease_txt = "Motor_kp_decrease.txt";
    const char *Motor_int_txt = "Motor_int.txt";
    const char *Limit_k_txt = "Limit_k.txt";

    // ---------- 从文件读取参数并初始化全局变量 ----------
    file_read_string(Motor_L_Kp1_txt, Motor_L_Kp1);               // 读取左电机Kp文件
    file_read_string(Motor_L_Ki1_txt, Motor_L_Ki1);               // 读取左电机Ki文件
    file_read_string(Motor_R_Kp1_txt, Motor_R_Kp1);           // 读取右电机Kp文件
    file_read_string(Motor_R_Ki1_txt, Motor_R_Ki1);           // 读取右电机Ki文件
    file_read_string(Motor_kp_increase_txt, Motor_kp_increase); // 读取速度增量文件
    file_read_string(Motor_kp_decrease_txt, Motor_kp_decrease); // 读取速度减量文件
    file_read_string(Motor_int_txt, Motor_Speed);            // 读取目标速度文件
    file_read_string(Limit_k_txt, Limit_k);                  // 读取限幅系数文件
   
    // 将字符串转换为数值，存入全局变量（供电机控制算法使用）
    Motor_PID_L.Kp = atof(Motor_L_Kp1);          // 左电机 Kp
    Motor_PID_L.Ki = atof(Motor_L_Ki1);          // 左电机 Ki
    Motor_PID_R.Kp = atof(Motor_R_Kp1);        // 右电机 Kp
    Motor_PID_R.Ki = atof(Motor_R_Ki1);        // 右电机 Ki
    kp_increase = atof(Motor_kp_increase);     // 速度增量（用于PID前馈或加速）
    kp_decrease = atof(Motor_kp_decrease);     // 速度减量（用于减速）
    Speed_Goal = atoi(Motor_Speed);            // 目标速度（整数）
    limit_k = atof(Limit_k);                   // 输出限幅系数

            // 每次循环先清屏，保证界面
    while (1) {
        switch (moflag) {
            case 1:
                // 返回箭头行
                menu_show_string(96, 0,   "<---");
                menu_show_string(96, 16,  "    ");
                menu_show_string(144, 128, "    ");
                // 静态标签和数值
                menu_show_string(0, 0,   "<--");
                menu_show_string(0, 16,  "Kpl:");
                menu_show_float(32, 16, Motor_PID_L.Kp, 2, 1);
                menu_show_string(0, 32,  "Kil:");
                menu_show_float(32, 32, Motor_PID_L.Ki, 2, 1);
                menu_show_string(0, 48,  "Kpr:");
                menu_show_float(32, 48, Motor_PID_R.Kp, 2, 1);
                menu_show_string(0, 64,  "Kir:");
                menu_show_float(32, 64, Motor_PID_R.Ki, 2, 1);
                menu_show_string(0, 80,  "Speed_IN:");
                menu_show_float(96, 80, kp_increase, 3, 0);
                menu_show_string(0, 96,  "kp_decrease:");
                menu_show_float(96, 96, kp_decrease, 2, 2);
                menu_show_string(0, 112, "Speed:");
                menu_show_int(48, 112, Speed_Goal, 3);
                menu_show_string(0, 128, "Limit_k:");
                menu_show_float(64, 128, limit_k, 1, 2);
                // 同步全局变量（实际上已经是最新，但保留原风格）
                Motor_PID_L.Kp = atof(Motor_L_Kp1);
                Motor_PID_L.Ki = atof(Motor_L_Ki1);
                Motor_PID_R.Kp = atof(Motor_R_Kp1);
                Motor_PID_R.Ki = atof(Motor_R_Ki1);
                kp_increase = atof(Motor_kp_increase);
                kp_decrease = atof(Motor_kp_decrease);
                Speed_Goal = atoi(Motor_Speed);
                limit_k = atof(Limit_k);
                break;

            case 2:
                menu_show_string(96, 0,   "    ");
                menu_show_string(96, 16,  "<---");
                menu_show_string(96, 32,  "    ");
                menu_show_string(0, 0,   "<--");
                menu_show_string(0, 16,  "Kpl:");
                menu_show_float(32, 16, Motor_PID_L.Kp, 2, 1);
                menu_show_string(0, 32,  "Kil:");
                menu_show_float(32, 32, Motor_PID_L.Ki, 2, 1);
                menu_show_string(0, 48,  "Kpr:");
                menu_show_float(32, 48, Motor_PID_R.Kp, 2, 1);
                menu_show_string(0, 64,  "Kir:");
                menu_show_float(32, 64, Motor_PID_R.Ki, 2, 1);
                menu_show_string(0, 80,  "Speed_IN:");
                menu_show_float(96, 80, kp_increase, 3, 0);
                menu_show_string(0, 96,  "kp_decrease:");
                menu_show_float(96, 96, kp_decrease, 2, 2);
                menu_show_string(0, 112, "Speed:");
                menu_show_int(48, 112, Speed_Goal, 3);
                menu_show_string(0, 144, "Limit_k:");
                menu_show_float(64, 144, limit_k, 1, 2);
                // 同步
                Motor_PID_L.Kp = atof(Motor_L_Kp1);
                Motor_PID_L.Ki = atof(Motor_L_Ki1);
                Motor_PID_R.Kp = atof(Motor_R_Kp1);
                Motor_PID_R.Ki = atof(Motor_R_Ki1);
                kp_increase = atof(Motor_kp_increase);
                kp_decrease = atof(Motor_kp_decrease);
                Speed_Goal = atoi(Motor_Speed);
                limit_k = atof(Limit_k);
                break;

            case 3:
                menu_show_string(96, 16,  "    ");
                menu_show_string(96, 32,  "<---");
                menu_show_string(144, 48, "    ");
                menu_show_string(0, 0,   "<--");
                menu_show_string(0, 16,  "Kpl:");
                menu_show_float(32, 16, Motor_PID_L.Kp, 2, 1);
                menu_show_string(0, 32,  "Kil:");
                menu_show_float(32, 32, Motor_PID_L.Ki, 2, 1);
                menu_show_string(0, 48,  "Kpr:");
                menu_show_float(32, 48, Motor_PID_R.Kp, 2, 1);
                menu_show_string(0, 64,  "Kir:");
                menu_show_float(32, 64, Motor_PID_R.Ki, 2, 1);
                menu_show_string(0, 80,  "Speed_IN:");
                menu_show_float(96, 80, kp_increase, 3, 0);
                menu_show_string(0, 96,  "kp_decrease:");
                menu_show_float(96, 96, kp_decrease, 2, 2);
                menu_show_string(0, 112, "Speed:");
                menu_show_int(48, 112, Speed_Goal, 3);
                menu_show_string(0, 128, "Limit_k:");
                menu_show_float(64, 128, limit_k, 1, 2);
                // 同步
                Motor_PID_L.Kp = atof(Motor_L_Kp1);
                Motor_PID_L.Ki = atof(Motor_L_Ki1);
                Motor_PID_R.Kp = atof(Motor_R_Kp1);
                Motor_PID_R.Ki = atof(Motor_R_Ki1);
                kp_increase = atof(Motor_kp_increase);
                kp_decrease = atof(Motor_kp_decrease);
                Speed_Goal = atoi(Motor_Speed);
                limit_k = atof(Limit_k);
                break;

            case 4:
                menu_show_string(96, 32,  "    ");
                menu_show_string(144, 48, "<---");
                menu_show_string(144, 64, "    ");
                menu_show_string(0, 0,   "<--");
                menu_show_string(0, 16,  "Kpl:");
                menu_show_float(32, 16, Motor_PID_L.Kp, 2, 1);
                menu_show_string(0, 32,  "Kil:");
                menu_show_float(32, 32, Motor_PID_L.Ki, 2, 1);
                menu_show_string(0, 48,  "Kpr:");
                menu_show_float(32, 48, Motor_PID_R.Kp, 2, 1);
                menu_show_string(0, 64,  "Kir:");
                menu_show_float(32, 64, Motor_PID_R.Ki, 2, 1);
                menu_show_string(0, 80,  "Speed_IN:");
                menu_show_float(96, 80, kp_increase, 3, 0);
                menu_show_string(0, 96,  "kp_decrease:");
                menu_show_float(96, 96, kp_decrease, 2, 2);
                menu_show_string(0, 112, "Speed:");
                menu_show_int(48, 112, Speed_Goal, 3);
                menu_show_string(0, 128, "Limit_k:");
                menu_show_float(64, 128, limit_k, 1, 2);
                // 同步
                Motor_PID_L.Kp = atof(Motor_L_Kp1);
                Motor_PID_L.Ki = atof(Motor_L_Ki1);
                Motor_PID_R.Kp = atof(Motor_R_Kp1);
                Motor_PID_R.Ki = atof(Motor_R_Ki1);
                kp_increase = atof(Motor_kp_increase);
                kp_decrease = atof(Motor_kp_decrease);
                Speed_Goal = atoi(Motor_Speed);
                limit_k = atof(Limit_k);
                break;

            case 5:
                menu_show_string(144, 48,  "    ");
                menu_show_string(144, 64,  "<---");
                menu_show_string(144, 80,  "    ");
                menu_show_string(0, 0,   "<--");
                menu_show_string(0, 16,  "Kpl:");
                menu_show_float(32, 16, Motor_PID_L.Kp, 2, 1);
                menu_show_string(0, 32,  "Kil:");
                menu_show_float(32, 32, Motor_PID_L.Ki, 2, 1);
                menu_show_string(0, 48,  "Kpr:");
                menu_show_float(32, 48, Motor_PID_R.Kp, 2, 1);
                menu_show_string(0, 64,  "Kir:");
                menu_show_float(32, 64, Motor_PID_R.Ki, 2, 1);
                menu_show_string(0, 80,  "Speed_IN:");
                menu_show_float(96, 80, kp_increase, 3, 0);
                menu_show_string(0, 96,  "kp_decrease:");
                menu_show_float(96, 96, kp_decrease, 2, 2);
                menu_show_string(0, 112, "Speed:");
                menu_show_int(48, 112, Speed_Goal, 3);
                menu_show_string(0, 128, "Limit_k:");
                menu_show_float(64, 128, limit_k, 1, 2);
                // 同步
                Motor_PID_L.Kp = atof(Motor_L_Kp1);
                Motor_PID_L.Ki = atof(Motor_L_Ki1);
                Motor_PID_R.Kp = atof(Motor_R_Kp1);
                Motor_PID_R.Ki = atof(Motor_R_Ki1);
                kp_increase = atof(Motor_kp_increase);
                kp_decrease = atof(Motor_kp_decrease);
                Speed_Goal = atoi(Motor_Speed);
                limit_k = atof(Limit_k);
                break;

            case 6:
                menu_show_string(144, 96,  "    ");
                menu_show_string(144, 64,  "    ");
                menu_show_string(144, 80,  "<---");
                menu_show_string(0, 0,   "<--");
                menu_show_string(0, 16,  "Kpl:");
                menu_show_float(32, 16, Motor_PID_L.Kp, 2, 1);
                menu_show_string(0, 32,  "Kil:");
                menu_show_float(32, 32, Motor_PID_L.Ki, 2, 1);
                menu_show_string(0, 48,  "Kpr:");
                menu_show_float(32, 48, Motor_PID_R.Kp, 2, 1);
                menu_show_string(0, 64,  "Kir:");
                menu_show_float(32, 64, Motor_PID_R.Ki, 2, 1);
                menu_show_string(0, 80,  "Speed_IN:");
                menu_show_float(96, 80, kp_increase, 3, 0);
                menu_show_string(0, 96,  "kp_decrease:");
                menu_show_float(96, 96, kp_decrease, 2, 2);
                menu_show_string(0, 112, "Speed:");
                menu_show_int(48, 112, Speed_Goal, 3);
                menu_show_string(0, 128, "Limit_k:");
                menu_show_float(64, 128, limit_k, 1, 2);
                // 同步
                Motor_PID_L.Kp = atof(Motor_L_Kp1);
                Motor_PID_L.Ki = atof(Motor_L_Ki1);
                Motor_PID_R.Kp = atof(Motor_R_Kp1);
                Motor_PID_R.Ki = atof(Motor_R_Ki1);
                kp_increase = atof(Motor_kp_increase);
                kp_decrease = atof(Motor_kp_decrease);
                Speed_Goal = atoi(Motor_Speed);
                limit_k = atof(Limit_k);
                break;

            case 7:
                menu_show_string(144, 112, "    ");
                menu_show_string(144, 80,  "    ");
                menu_show_string(144, 96,  "<---");
                menu_show_string(0, 0,   "<--");
                menu_show_string(0, 16,  "Kpl:");
                menu_show_float(32, 16, Motor_PID_L.Kp, 2, 1);
                menu_show_string(0, 32,  "Kil:");
                menu_show_float(32, 32, Motor_PID_L.Ki, 2, 1);
                menu_show_string(0, 48,  "Kpr:");
                menu_show_float(32, 48, Motor_PID_R.Kp, 2, 1);
                menu_show_string(0, 64,  "Kir:");
                menu_show_float(32, 64, Motor_PID_R.Ki, 2, 1);
                menu_show_string(0, 80,  "Speed_IN:");
                menu_show_float(96, 80, kp_increase, 3, 0);
                menu_show_string(0, 96,  "kp_decrease:");
                menu_show_float(96, 96, kp_decrease, 2, 2);
                menu_show_string(0, 112, "Speed:");
                menu_show_int(48, 112, Speed_Goal, 3);
                menu_show_string(0, 128, "Limit_k:");
                menu_show_float(64, 128, limit_k, 1, 2);
                // 同步
                Motor_PID_L.Kp = atof(Motor_L_Kp1);
                Motor_PID_L.Ki = atof(Motor_L_Ki1);
                Motor_PID_R.Kp = atof(Motor_R_Kp1);
                Motor_PID_R.Ki = atof(Motor_R_Ki1);
                kp_increase = atof(Motor_kp_increase);
                kp_decrease = atof(Motor_kp_decrease);
                Speed_Goal = atoi(Motor_Speed);
                limit_k = atof(Limit_k);
                break;

            case 8:
                menu_show_string(144, 96,  "    ");
                menu_show_string(144, 128, "    ");
                menu_show_string(144, 112, "<---");
                menu_show_string(0, 0,   "<--");
                menu_show_string(0, 16,  "Kpl:");
                menu_show_float(32, 16, Motor_PID_L.Kp, 2, 1);
                menu_show_string(0, 32,  "Kil:");
                menu_show_float(32, 32, Motor_PID_L.Ki, 2, 1);
                menu_show_string(0, 48,  "Kpr:");
                menu_show_float(32, 48, Motor_PID_R.Kp, 2, 1);
                menu_show_string(0, 64,  "Kir:");
                menu_show_float(32, 64, Motor_PID_R.Ki, 2, 1);
                menu_show_string(0, 80,  "Speed_IN:");
                menu_show_float(96, 80, kp_increase, 3, 0);
                menu_show_string(0, 96,  "kp_decrease:");
                menu_show_float(96, 96, kp_decrease, 2, 2);
                menu_show_string(0, 112, "Speed:");
                menu_show_int(48, 112, Speed_Goal, 3);
                menu_show_string(0, 128, "Limit_k:");
                menu_show_float(64, 128, limit_k, 1, 2);
                // 同步
                Motor_PID_L.Kp = atof(Motor_L_Kp1);
                Motor_PID_L.Ki = atof(Motor_L_Ki1);
                Motor_PID_R.Kp = atof(Motor_R_Kp1);
                Motor_PID_R.Ki = atof(Motor_R_Ki1);
                kp_increase = atof(Motor_kp_increase);
                kp_decrease = atof(Motor_kp_decrease);
                Speed_Goal = atoi(Motor_Speed);
                limit_k = atof(Limit_k);
                break;

            case 9:
                menu_show_string(144, 112, "    ");
                menu_show_string(96, 0,    "    ");
                menu_show_string(144, 128, "<---");
                menu_show_string(0, 0,   "<--");
                menu_show_string(0, 16,  "Kpl:");
                menu_show_float(32, 16, Motor_PID_L.Kp, 2, 1);
                menu_show_string(0, 32,  "Kil:");
                menu_show_float(32, 32, Motor_PID_L.Ki, 2, 1);
                menu_show_string(0, 48,  "Kpr:");
                menu_show_float(32, 48, Motor_PID_R.Kp, 2, 1);
                menu_show_string(0, 64,  "Kir:");
                menu_show_float(32, 64, Motor_PID_R.Ki, 2, 1);
                menu_show_string(0, 80,  "Speed_IN:");
                menu_show_float(96, 80, kp_increase, 3, 0);
                menu_show_string(0, 96,  "kp_decrease:");
                menu_show_float(96, 96, kp_decrease, 2, 2);
                menu_show_string(0, 112, "Speed:");
                menu_show_int(48, 112, Speed_Goal, 3);
                menu_show_string(0, 128, "Limit_k:");
                menu_show_float(64, 128, limit_k, 1, 2);
                // 同步
                Motor_PID_L.Kp = atof(Motor_L_Kp1);
                Motor_PID_L.Ki = atof(Motor_L_Ki1);
                Motor_PID_R.Kp = atof(Motor_R_Kp1);
                Motor_PID_R.Ki = atof(Motor_R_Ki1);
                kp_increase = atof(Motor_kp_increase);
                kp_decrease = atof(Motor_kp_decrease);
                Speed_Goal = atoi(Motor_Speed);
                limit_k = atof(Limit_k);
                break;
        }

        // ---- 获取按键 ----
        int8_t KeyNum = Key_GetNum();  // 返回：1上，2下，3短按（减），4长按（加）

        // 上键：光标上移，循环范围 1~9
        if (KeyNum == 1) {
            moflag--;
            if (moflag == 0) moflag = 9;
        }

        // 下键：光标下移，循环范围 1~9
        if (KeyNum == 2) {
            moflag++;
            if (moflag == 10) moflag = 1;
        }

        // 短按（减）：根据当前光标位置减少对应的参数值
        if (KeyNum == 3) {
            switch (moflag) {
                case 2:   // 左电机 Kp 减少
                    Motor_PID_L.Kp -= 0.2;
                    sprintf(Motor_L_Kp1, "%f", Motor_PID_L.Kp);
                    file_write_string(Motor_L_Kp1_txt, Motor_L_Kp1);
                    break;
                case 3:   // 左电机 Ki 减少
                    Motor_PID_L.Ki -= 0.0001;
                    sprintf(Motor_L_Ki1, "%f", Motor_PID_L.Ki);
                    file_write_string(Motor_L_Ki1_txt, Motor_L_Ki1);
                    break;
                case 4:   // 右电机 Kp 减少
                    Motor_PID_R .Kp -= 0.2;
                    sprintf(Motor_R_Kp1, "%f", Motor_PID_R .Kp);
                    file_write_string(Motor_R_Kp1_txt, Motor_R_Kp1);
                    break;
                case 5:   // 右电机 Ki 减少
                    Motor_PID_R.Ki -= 0.0001;
                    sprintf(Motor_R_Ki1, "%f", Motor_PID_R.Ki);
                    file_write_string(Motor_R_Ki1_txt, Motor_R_Ki1);
                    break;
                case 6:   // 速度增量减少
                    kp_increase -= 10;
                    sprintf(Motor_kp_increase, "%f", kp_increase);
                    file_write_string(Motor_kp_increase_txt, Motor_kp_increase);
                    break;
                case 7:   // 速度减量减少
                    kp_decrease -= 0.01;
                    sprintf(Motor_kp_decrease, "%f", kp_decrease);
                    file_write_string(Motor_kp_decrease_txt, Motor_kp_decrease);
                    break;
                case 8:   // 目标速度减少
                    Speed_Goal -= 5;
                    sprintf(Motor_Speed, "%d", Speed_Goal);
                    file_write_string(Motor_int_txt, Motor_Speed);
                    break;
                case 9:   // 限幅系数减少
                    limit_k -= 0.02;
                    sprintf(Limit_k, "%f", limit_k);
                    file_write_string(Limit_k_txt, Limit_k);
                    break;
                case 1:   // 光标在返回项时，短按退出菜单
                    menu_clear();
                    return 0;
            }
            // 修改后重新从文件读取，确保全局变量与文件一致（也可直接用内存值）
            file_read_string(Motor_L_Kp1_txt, Motor_L_Kp1); Motor_PID_L.Kp = atof(Motor_L_Kp1);
            file_read_string(Motor_L_Ki1_txt, Motor_L_Ki1); Motor_PID_L.Ki = atof(Motor_L_Ki1);
            file_read_string(Motor_R_Kp1_txt, Motor_R_Kp1); Motor_PID_R .Kp = atof(Motor_R_Kp1);
            file_read_string(Motor_R_Ki1_txt, Motor_R_Ki1); Motor_PID_R.Ki = atof(Motor_R_Ki1);
            file_read_string(Motor_kp_increase_txt, Motor_kp_increase); kp_increase = atof(Motor_kp_increase);
            file_read_string(Motor_kp_decrease_txt, Motor_kp_decrease); kp_decrease = atof(Motor_kp_decrease);
            file_read_string(Motor_int_txt, Motor_Speed); Speed_Goal = atoi(Motor_Speed);
            file_read_string(Limit_k_txt, Limit_k); limit_k = atof(Limit_k);
        }

        // 长按（加）：根据当前光标位置增加对应的参数值
        if (KeyNum == 4) {
            switch (moflag) {
                case 2:   // 左电机 Kp 增加
                    Motor_PID_L.Kp += 0.2;
                    sprintf(Motor_L_Kp1, "%f", Motor_PID_L.Kp);
                    file_write_string(Motor_L_Kp1_txt, Motor_L_Kp1);
                    break;
                case 3:   // 左电机 Ki 增加
                    Motor_PID_L.Ki += 0.0001;
                    sprintf(Motor_L_Ki1, "%f", Motor_PID_L.Ki);
                    file_write_string(Motor_L_Ki1_txt, Motor_L_Ki1);
                    break;
                case 4:   // 右电机 Kp 增加
                    Motor_PID_R .Kp += 0.2;
                    sprintf(Motor_R_Kp1, "%f", Motor_PID_R .Kp);
                    file_write_string(Motor_R_Kp1_txt, Motor_R_Kp1);
                    break;
                case 5:   // 右电机 Ki 增加
                    Motor_PID_R.Ki += 0.0001;
                    sprintf(Motor_R_Ki1, "%f", Motor_PID_R.Ki);
                    file_write_string(Motor_R_Ki1_txt, Motor_R_Ki1);
                    break;
                case 6:   // 速度增量增加
                    kp_increase += 10;
                    sprintf(Motor_kp_increase, "%f", kp_increase);
                    file_write_string(Motor_kp_increase_txt, Motor_kp_increase);
                    break;
                case 7:   // 速度减量增加
                    kp_decrease += 0.01;
                    sprintf(Motor_kp_decrease, "%f", kp_decrease);
                    file_write_string(Motor_kp_decrease_txt, Motor_kp_decrease);
                    break;
                case 8:   // 目标速度增加
                    Speed_Goal += 5;
                    sprintf(Motor_Speed, "%d", Speed_Goal);
                    file_write_string(Motor_int_txt, Motor_Speed);
                    break;
                case 9:   // 限幅系数增加
                    limit_k += 0.02;
                    sprintf(Limit_k, "%f", limit_k);
                    file_write_string(Limit_k_txt, Limit_k);
                    break;
                case 1:   // 光标在返回项时，长按也退出菜单
                    menu_clear();
                    return 0;
            }
            // 同样，修改后重新读取文件同步数据
            file_read_string(Motor_L_Kp1_txt, Motor_L_Kp1); Motor_PID_L.Kp = atof(Motor_L_Kp1);
            file_read_string(Motor_L_Ki1_txt, Motor_L_Ki1); Motor_PID_L.Ki = atof(Motor_L_Ki1);
            file_read_string(Motor_R_Kp1_txt, Motor_R_Kp1); Motor_PID_R .Kp = atof(Motor_R_Kp1);
            file_read_string(Motor_R_Ki1_txt, Motor_R_Ki1); Motor_PID_R.Ki = atof(Motor_R_Ki1);
            file_read_string(Motor_kp_increase_txt, Motor_kp_increase); kp_increase = atof(Motor_kp_increase);
            file_read_string(Motor_kp_decrease_txt, Motor_kp_decrease); kp_decrease = atof(Motor_kp_decrease);
            file_read_string(Motor_int_txt, Motor_Speed); Speed_Goal = atoi(Motor_Speed);
            file_read_string(Limit_k_txt, Limit_k); limit_k = atof(Limit_k);
        }
        usleep(10000); // 10ms
    } // while(1) 循环结束
}

int menu2_Angle_PID() {
    uint8_t moflag = 1;   // 1:返回, 2:Kp, 3:Kd, 4:MaxOut
    char angle_Kp_buf[50] = {0};
    char angle_Kd_buf[50] = {0};
    char angle_maxout_buf[50] = {0};

    const char *angle_Kp_txt = "Angle_Kp.txt";
    const char *angle_Kd_txt = "Angle_Kd.txt";
    const char *angle_maxout_txt = "Angle_MaxOut.txt";

    // 读取文件
    file_read_string(angle_Kp_txt, angle_Kp_buf);
    file_read_string(angle_Kd_txt, angle_Kd_buf);
    file_read_string(angle_maxout_txt, angle_maxout_buf);

    Angle_PID.Kp = atof(angle_Kp_buf);
    Angle_PID.Kd = atof(angle_Kd_buf);
    Angle_PID.MaxOut = atof(angle_maxout_buf);

    // 初次绘制全部界面（不清屏，避免闪烁）
    while (1) {
        // 根据 moflag 绘制完整界面
        switch (moflag) {
            case 1:
                menu_show_string(96, 0,   "<---");
                menu_show_string(96, 16,  "    ");
                menu_show_string(96, 32,  "    ");
                // 静态标签
                menu_show_string(0, 0,   "<--");
                menu_show_string(0, 16,  "Kp:");
                menu_show_float(32, 16, Angle_PID.Kp, 2, 3);
                menu_show_string(0, 32,  "Kd:");
                menu_show_float(32, 32, Angle_PID.Kd, 2, 3);
                menu_show_string(0, 48,  "MaxOut:");
                menu_show_float(64, 48, Angle_PID.MaxOut, 3, 0);
                break;

            case 2:
                menu_show_string(96, 0,   "    ");
                menu_show_string(96, 16,  "<---");
                menu_show_string(96, 32,  "    ");
                menu_show_string(0, 0,   "<--");
                menu_show_string(0, 16,  "Kp:");
                menu_show_float(32, 16, Angle_PID.Kp, 2, 3);
                menu_show_string(0, 32,  "Kd:");
                menu_show_float(32, 32, Angle_PID.Kd, 2, 3);
                menu_show_string(0, 48,  "MaxOut:");
                menu_show_float(64, 48, Angle_PID.MaxOut, 3, 0);
                break;

            case 3:
                menu_show_string(96, 0,   "    ");
                menu_show_string(96, 16,  "    ");
                menu_show_string(96, 32,  "<---");
                menu_show_string(0, 0,   "<--");
                menu_show_string(0, 16,  "Kp:");
                menu_show_float(32, 16, Angle_PID.Kp, 2, 3);
                menu_show_string(0, 32,  "Kd:");
                menu_show_float(32, 32, Angle_PID.Kd, 2, 3);
                menu_show_string(0, 48,  "MaxOut:");
                menu_show_float(64, 48, Angle_PID.MaxOut, 3, 0);
                break;

            case 4:
                menu_show_string(96, 0,   "    ");
                menu_show_string(96, 16,  "    ");
                menu_show_string(96, 32,  "    ");
                menu_show_string(96, 48,  "<---");
                menu_show_string(0, 0,   "<--");
                menu_show_string(0, 16,  "Kp:");
                menu_show_float(32, 16, Angle_PID.Kp, 2, 3);
                menu_show_string(0, 32,  "Kd:");
                menu_show_float(32, 32, Angle_PID.Kd, 2, 3);
                menu_show_string(0, 48,  "MaxOut:");
                menu_show_float(64, 48, Angle_PID.MaxOut, 3, 0);
                break;
        }

        int8_t KeyNum = Key_GetNum();

        // 上下移动光标
        if (KeyNum == 1) { moflag--; if (moflag == 0) moflag = 4; }
        if (KeyNum == 2) { moflag++; if (moflag == 5) moflag = 1; }

        // 短按（减）
        if (KeyNum == 3) {
            switch (moflag) {
                case 2: Angle_PID.Kp -= 0.1; sprintf(angle_Kp_buf, "%f", Angle_PID.Kp); file_write_string(angle_Kp_txt, angle_Kp_buf); break;
                case 3: Angle_PID.Kd -= 0.1; sprintf(angle_Kd_buf, "%f", Angle_PID.Kd); file_write_string(angle_Kd_txt, angle_Kd_buf); break;
                case 4: Angle_PID.MaxOut -= 5; sprintf(angle_maxout_buf, "%f", Angle_PID.MaxOut); file_write_string(angle_maxout_txt, angle_maxout_buf); break;
                case 1: menu_clear(); return 0;
            }
            // 重新读取文件同步
            file_read_string(angle_Kp_txt, angle_Kp_buf); Angle_PID.Kp = atof(angle_Kp_buf);
            file_read_string(angle_Kd_txt, angle_Kd_buf); Angle_PID.Kd = atof(angle_Kd_buf);
            file_read_string(angle_maxout_txt, angle_maxout_buf); Angle_PID.MaxOut = atof(angle_maxout_buf);
        }

        // 长按（加）
        if (KeyNum == 4) {
            switch (moflag) {
                case 2: Angle_PID.Kp += 0.1; sprintf(angle_Kp_buf, "%f", Angle_PID.Kp); file_write_string(angle_Kp_txt, angle_Kp_buf); break;
                case 3: Angle_PID.Kd += 0.1; sprintf(angle_Kd_buf, "%f", Angle_PID.Kd); file_write_string(angle_Kd_txt, angle_Kd_buf); break;
                case 4: Angle_PID.MaxOut += 5; sprintf(angle_maxout_buf, "%f", Angle_PID.MaxOut); file_write_string(angle_maxout_txt, angle_maxout_buf); break;
                case 1: menu_clear(); return 0;
            }
            file_read_string(angle_Kp_txt, angle_Kp_buf); Angle_PID.Kp = atof(angle_Kp_buf);
            file_read_string(angle_Kd_txt, angle_Kd_buf); Angle_PID.Kd = atof(angle_Kd_buf);
            file_read_string(angle_maxout_txt, angle_maxout_buf); Angle_PID.MaxOut = atof(angle_maxout_buf);
        }

        usleep(10000); // 10ms 延时，降低 CPU 占用
    }
}
void Load_All_Params(void)
{
    char buf[50];
    if (file_read_string("Motor_L_Kp1.txt", buf) == 0) Motor_PID_L.Kp = atof(buf);
    if (file_read_string("Motor_L_Ki1.txt", buf) == 0) Motor_PID_L.Ki = atof(buf);
    if (file_read_string("Motor_R_Kp1.txt", buf) == 0) Motor_PID_R.Kp = atof(buf);
    if (file_read_string("Motor_R_Ki1.txt", buf) == 0) Motor_PID_R.Ki = atof(buf);
    if (file_read_string("Motor_kp_increase.txt", buf) == 0) kp_increase = atof(buf);
    if (file_read_string("Motor_kp_decrease.txt", buf) == 0) kp_decrease = atof(buf);
    if (file_read_string("Motor_int.txt", buf) == 0) Speed_Goal = atoi(buf);
    if (file_read_string("Limit_k.txt", buf) == 0) limit_k = atof(buf);
}