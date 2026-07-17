#define lq_compile_assert(expr, msg)
#include"Gyro.hpp"
#include"Circle.h"
#include<iostream>
using namespace std;
//状态机核心标志位，用来记录当前小车处于圆环任务的哪个状态
Circle_t circle_type=Circle_NULL;  
//实时计算的转角累积量
static float Cyaw=0;
//进入圆环那一瞬间的初始角度，计算小车进入圆环之后转了多少度
static float Check_yaw=0;
//视觉检测到圆环特征(如L型拐点)时不立刻切换状态，而是连续3帧图像都看到这个特征
uint8 Circle_fps_th=3;
//入环阶段的角度阈值，当小车正在入环时，它需要一直保持入环的特殊控制，知道陀螺仪显示小车已经转过了60度
float Circle_IN_yaw=120.0f;
//只有当Cyaw超过160度之后，程序才允许Circle_Check去寻找出环的特征点
float Circle_Out_yaw=160.0f;

//识别圆环元素
void Circle_Check(void)
{
    //如果全局检测开关开启，且当前不处于任何圆环状态中
    if(All_Check&&Circle_NULL==circle_type){
        //左边界为直线，右侧检测到一个外角点，并且右侧角点位置非常靠下
        if(is_straightL&&L_Out==RightP.LP&&RightP.ID<10){
            circle_type=Circle_RIN;
            Check_yaw=yaw_angle;
            //Buzzer_OpenClose(1);
        }
        //右边界为直线，左侧检测到一个外角点，并且左侧角点位置非常靠下
        else if(is_straightR&&L_Out==LeftP.LP&&LeftP.ID<10){
            circle_type=Circle_LIN;
            Check_yaw=yaw_angle;
            //Buzzer_OpenClose(1);
        }
        //以上两种情况都不符合的话就是没有circle
        else{
            circle_type=Circle_NULL;
        }
    }
    //又或者是在圆环当中，但是又转了360度走出圆环
    else{
        if((is_straightR&&is_straightL)||fabs(Check_yaw-yaw_angle)>360.0f){
            circle_type=Circle_NULL;
        }
    }
}

void Circle_Loop(void)
{
    Circle_Check();
    static uint8 Lflag=0,Rflag=0;
    static uint8 L_out_flag=0,R_out_flag=0;
    uint8 flag1=0;
    static uint8 fps=0;
    switch(circle_type){
        //左入环阶段
        case Circle_LIN:
            //确认爬虫已经抓到了线(Rep_Lnum>0)
            if(Lflag==0&&Rep_Lnum<=10)
                Lflag=1;
            //LRType=MID_Right;//强制锁定右单边巡线，忽略乱掉的左边
            //状态跳转条件：
            //当左边爬到的点数足够多(>20),且拐点消失了(L_NULL==LeftP.LP),
            //并且左边线不再是直的(!is_straightL),说明车身已经进入圆弧
            if(Lflag&&(Rep_Lnum>45&&L_NULL==LeftP.LP&&!is_straightL)){
                Lflag=0;    //清除标志状态位供下次使用
                circle_type=Circle_L;  //正式进入环内状态
                Cyaw=yaw_angle;    //再次校准陀螺仪角度锚点
            }
            break;
        //右入环阶段
        case Circle_RIN:
            //确认爬虫已经抓到了线(Rep_Rnum>0)
            if(Rflag==0&&Rep_Rnum<=10)
                Rflag=1;
            //LRType=MID_Left;//强制锁定左单边巡线，忽略乱掉的右边
            //状态跳转条件：
            //当右边爬到的点数足够多(>20),且拐点消失了(L_NULL==LeftP.LP),
            //并且左边线不再是值得(!is_straightR),说明车身已经进入圆弧
            if(Rflag&&(Rep_Rnum>45&&L_NULL==RightP.LP&&!is_straightR)){
                Rflag=0;   //清楚标志状态位供下次使用
                circle_type=Circle_R;  //正式进入环内状态
                Cyaw=yaw_angle;   //再次校准陀螺仪角度锚点
            }
            break;
        //环内阶段
        case Circle_L:
            flag1=0;
            LRType=MID_Left;//强制锁定左单边巡线，忽略乱掉的右边

            //寻找环内特定的"断点"或特征，结合角度判断
            if(0==Lflag)
                Lflag=1;
            //cout<<"flag1:  "<<flag1<<" Rep_Lnum: "<<Rep_Lnum<<" is_straightR: "<<is_straightR<<endl;
             //如果左边重新出现了线，并且不是直的，说明快找到出口了
            //如果右边重新出现了线，并且不是直的，说明快找到出口了
            if(flag1==0&&Rep_Rnum>30&&Lflag&&!is_straightR)
                flag1=1;
            //旋转角度累计超过了Circle_IN_yaw(比如60度)
            //且满足特征flag1，认为到了准备出环的时机
            if(fabs(yaw_angle-Cyaw)>Circle_IN_yaw&&flag1){
                circle_type=Circle_LOut;  //切换到出环阶段
                L_out_flag=0;
                fps=0;
            }
            break;
        case Circle_R:
            flag1=0;
            LRType=MID_Right;//强制锁定右单边巡线，忽略乱掉的左边

            //寻找环内特定的“断点”或特征，结合角度判断
            if(Rflag==0)
                Rflag=1;
            //cout<<"flag1:  "<<flag1<<" Rep_Rnum: "<<Rep_Rnum<<" is_straightL: "<<is_straightL<<endl;
            //如果右边重新出现了线，并且不是直的，说明快找到出口了
            if(flag1==0&&Rep_Lnum>30&&Rflag&&!is_straightL){
                flag1=1;
            }

            //旋转角度累计超过了Circle_IN_yaw(比如60度)
            //且满足特征flag1，认为到了准备出环的时机
            if(fabs(yaw_angle-Cyaw)>Circle_IN_yaw&&flag1){
                circle_type=Circle_ROut;  //切换到出环阶段
                R_out_flag=0;
                fps=0;
            }
            break;
        //准备出环
        case Circle_LOut:
            //寻找出环的角点特征
            if(0==L_out_flag&&L_NULL!=RightP.LP&&RightP.ID<20)
                L_out_flag=1;
            LRType=MID_Left;  //暂时还是贴着左边跑
            if(L_out_flag){
                //LRType=MID_Left;
                //寻找环外的直到(回归信号)
                if(is_straightR){
                    LRType=MID_Right;  //看到右边有直道，尝试切换
                    fps++;  //帧数滤波，防止误判
                }
                //确认彻底出环
                if(fps>Circle_fps_th&is_straightR){//确认连续多帧看到了左边的直道
                    if(is_straightR) //再次确认左边线是直的
                        LRType=MID_Right;  //切换控制参考系
                    circle_type=Circle_NULL;  //切回普通模式
                    //Buzzer_OpenClose(0);
                    L_out_flag=0;
                    fps=0;
                }
            }
            break;
        case Circle_ROut:
            //寻找出环的角点特征
            if(R_out_flag==0&&L_NULL!=LeftP.LP&&LeftP.ID<20)
                R_out_flag=1;
            
            LRType=MID_Right; //暂时还是贴着右边跑
            if(R_out_flag){
                //LRType=MID_Right;
                //寻找环外的直到(回归信号)
                if(is_straightL){
                    LRType=MID_Left; //看到右边有直道，尝试切换
                    fps++;   //帧数滤波，防止误判
                }
                //确认彻底出环
                if(fps>Circle_fps_th&&is_straightL){  //确认连续多帧看到了左边的直道
                    if(is_straightL)  //再次确认左边线是直的
                        LRType=MID_Left;  //切换控制参考系
                    circle_type=Circle_NULL;//切回普通模式
                    //Buzzer_OpenClose(0);
                    R_out_flag=0;
                    fps=0;
                }
            }
            break;
        //最后什么都没有就直接将所有变量进行初始化
        default:
            Lflag=0;
            Rflag=0;
            L_out_flag=0;
            R_out_flag=0;
            fps=0;
            break;
    }
    //cout<<"fabs(yaw_angle-Cyaw):  "<<fabs(yaw_angle-Cyaw)<<endl;
}