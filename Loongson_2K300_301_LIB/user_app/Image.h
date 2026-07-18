/*
*vision.h
*vision part of the project
*created by lyc on 2026/03/20
*Copyright (c) 2026 by lyc, All Rights Reserved.
*/
#ifndef CODE_IMAGE_H    //include保护
#define CODE_IMAGE_H

#include <cstdint>
#include <cassert>// 引入标准断言库
//宏定义暂时消除部分环境问题
typedef uint8_t uint8;

//图像规格定义
#define IMG_W 320       //宽320
#define IMG_H 240        //高240
#define MAX_POINTS 160   //最大搜索点量
#define APPROX_NUM 10   //切线拟合距离
#define MID_DIS    22.5f//中线平移距离
#define RAD_TO_DEG 57.29578f
//有关图像边界的宏定义

//宏定义暂时消除部分环境问题
typedef uint8_t uint8;
typedef unsigned short uint16;
typedef signed char    int8;
typedef unsigned int int32;
//L型角相关定义与枚举
typedef enum{
    L_Out=0,
    L_In,
    L_NULL
}LPoint_t;

typedef struct{
    LPoint_t LP;
    uint8 ID;
}LP_t;

typedef enum{
    MID_Left=0,
    MID_Ramp,
    MID_Right,
}Mid_LineType;

//extern uint8_t* Image_Used;


extern LP_t LeftP,RightP; //左边道路和右边道路的判断
extern Mid_LineType LRType;  //判断中线是左还是右

//存放当前正在处理的图像
extern uint8 Image_Used[IMG_H][IMG_W];

extern uint8 OSTU_threshold;         //大津法算出来的阈值
extern uint8 OSTU_threshold_Add;    //阈值偏差，过滤背景噪声

//巡线相关参数
extern int Best_X;          //最长白列的X坐标(巡线起始点)
extern int Max_White_Height; //最长白列连续高度
extern int Standard_Width; //赛道在图像底部的像素宽度(用于补线)
extern int Block_Size;       //局部自适应窗口大小
extern int Clip_Value;       //经验值(通常4-10)，让阈值稍微比平均亮度黑一点
extern float img_k;     //逆透视修正系数
extern uint8 starty;        //搜索的起始y坐标
extern uint8 Find_LineNum;  //寻找边界的最大行数限制，防止过度所搜导致的误识别，尤其是在环岛状态下
extern int Straight_TH;     //直线判断阈值，单位为像素，当拟合的直线与实际边界点的平均距离小于该值，则认为边界是直线
extern float conf_MAXL;      //左边线角点置信度，单位为0-1，越大表示越有可能是角点
extern float conf_MAXR;      //右边线角点置信度，单位为0-1，越大表示越有可能是角点
extern int ANGLE_DIS;       //角点之间的最小距离，单位为像素，防止过于密集的角点导致误识别
extern int Base_see;        //调这个  //基础速度，单位为mm/s
extern int Tsee;            //目标速度，单位为mm/s
extern int Blue_Size;        //蓝色路径边界点数量，单位为点
extern volatile float Turn_R;//转弯半径，单位为像素，正值表示右转，负值表示左转，绝对值越大表示转弯越缓
extern uint8 CBHBUM;         //丢线保护标志，0表示不丢线保护，1表示丢线保护中
extern uint8 Base_starty;   //丢线保护起始行，单位为像素，通常设置在巡线起始点附近，例如95-110之间，根据实际情况调整
extern uint8 Check_Stop_TH; //丢线保护阈值，单位为像素，当连续丢线行数超过该值时，触发丢线保护机制，例如15-30之间，根据实际情况调整
extern uint8 Stop_CBH_TH;   //丢线保护的CBH阈值，单位为0-255，当CBH值超过该阈值时，认为当前行丢线，例如40-60之间，根据实际情况调整
extern uint8 Stop_car;       //丢线保护触发后是否停车，0表示不停车，1表示停车，根据实际需求设置
extern int Long_WhiteLine_X; //最长白线的X坐标，单位为像素，根据实际情况调整，例如设置在图像底部中央区域的范围内（50-110之间）
extern int Long_WhiteLine_Y; //最长白线的Y坐标，单位为像素，根据实际情况调整，例如设置在图像底部附近（90-110之间）
extern uint8 Long_WhiteLine; //
extern uint8 LossTH;        //丢线保护的丢线行数阈值，单位为行，当连续丢线行数超过该值时，触发丢线保护机制，例如20-40之间，根据实际情况调整
extern uint8 Car_go;         //
extern uint8 SpeedChangeH;//0:禁用  1：速度决策  2：直道速度决策 3:速度决策（无直道）
extern uint8 LongWhite_Speed; //
extern float straight_SpeedK;
extern float WhiteLine_SpeedK;
extern float dist;           //
const float K_CONTROL=150.0f;
//环岛相关参数
extern uint8 LRob;
extern uint8 RRob;
//边界与中线数据
//使用int储存像素坐标更节省内存[0-159]
extern int Left_Edge[MAX_POINTS][2];       //左边界坐标
extern int Right_Edge[MAX_POINTS][2];      //右边界坐标

extern float (*Center_Line)[2];     //中心线数组
extern int *Center_Line_Num;        //中线点数

extern float Blue_Left_Line[MAX_POINTS][2];    // 原始蓝色路径左边界坐标数组 [点索引][0=X坐标,1=Y坐标]
extern float Blue_Right_Line[MAX_POINTS][2];   // 原始蓝色路径右边界坐标数组 [点索引][0=X坐标,1=Y坐标]

extern float Left_Mid_Line[MAX_POINTS][2];  //左边中线坐标数组
extern int L_Mid_Num;    //左边中线点数量
extern float Right_Mid_Line[MAX_POINTS][2]; //右边中线坐标数组
extern int R_Mid_Num;    //右边中线点数量

extern float Left_Angel[MAX_POINTS];//左边界角点坐标数组，单位为像素坐标
extern float Right_Angel[MAX_POINTS];//右边界角点坐标数组，同理

extern float Nms_Left_Angel[MAX_POINTS];     //左边界角点置信度数组，单位为0-1，越大表示越有可能是角点
extern float Nms_Right_Angel[MAX_POINTS];    //右边界角点置信度数组，同上
// 路径边界备用坐标数组（用于拐角识别/滤波，避免修改原始坐标）
extern float Rep_Left_Line[MAX_POINTS][2];     // 左侧路径边界备用坐标数组 [点索引][0=X坐标,1=Y坐标]
extern int Rep_Lnum;              // RepLeft_Line数组中有效坐标点的数量（初始值为最大点数，后续更新为实际有效数）
extern float Rep_Right_Line[MAX_POINTS][2];    // 右侧路径边界备用坐标数组 [点索引][0=X坐标,1=Y坐标]
extern int Rep_Rnum;              // RepRight_Line数组中有效坐标点的数量（初始值为最大点数，后续更新为实际有效数）

extern float (*mid_pts)[2];  //指向当前使用的中线坐标数组的指针，根据左右边线的情况进行赋值
extern int *MidnumPts;   //指向当前使用的中线点数量的指针，根据左右边线的情况进行赋值
extern float Mid_Line[MAX_POINTS][2];   //当前使用的中线坐标数组，根据左右边线的情况进行赋值
extern int MidNum;

extern float Left_Line[MAX_POINTS][2];
extern int Left_Num;
extern float Right_Line[MAX_POINTS][2];
extern int Right_Num;
//标志位
extern bool Left_Lost_Flag;     //左边线丢失标志
extern bool Right_Lost_Flag;    //右边线丢失标志
extern bool All_Lost_Flag;      //全丢标志(丢线保护)

extern int left_lost_flag[IMG_H];    //按行进行丢线标识
extern int right_lost_flag[IMG_H];   //丢线置1，不丢线置0

extern bool is_straightL;         //左边线是否为直线标志位
extern bool is_straightR;         //右边线是否为直线标志位

extern uint8 All_Check;            //全局扫描开关，确定当前图像不是乱码、没有丢线、具备基本的赛道形状时，置为1
extern uint8 check_flag;           //扫描标志位，正在扫描时置1，扫描结束后置0
//边界丢线数
extern int right_lost_times;
extern int left_lost_times;
extern int both_lost_times;  //双丢线数
//常规边界起始点
extern int boundry_start_left;
extern int boundry_start_right;
//环岛状态
extern int right_island_flag;  //1表示检测到右环岛，0表示无
extern int left_island_flag;   //1表示检测到左环岛
extern int island_state;

extern float Dynamic_Mid_Offset;
//函数声明
//工具类函数
//uint16 CBH(uint8 a,uint8 b);               //
#ifdef __cplusplus
extern "C" {
#endif

// 强制显式使用 unsigned char
unsigned short CBH(unsigned char a, unsigned char b);

#ifdef __cplusplus
}
#endif
float residualSum(float pts[][2],int n);  //
void get_highest(int*LWX,int*LWY);        //
float fast_sqrt(float num);               //快速开根号，返回值为float型
float fast_invsqrt(float num);            //Quake III 快速平方根倒数
void blur_points(float pts_in[][2],int num,float pts_out[][2]);
void resample_points(float pts_in[][2],int num1,float pts_out[][2],int*num2);
void local_angel_points(float pts_in[][2],int num,float angle_out[]);
void nms_angle(float angle_in[],int num,float angle_out[]);

//大津法相关函数
void get_OSTU(void);  //大津法计算阈值
//巡线相关函数
static inline int updata_sum(int sum,int x,int y,int half,int dirx,int diry);
void findline_lefthand_adaptive(int x,int y,float pts[][2],int *num);
void findline_righthand_adaptive(int x,int y,float pts[][2],int *num);
void track_leftline(float pts_in[][2],int start,int end,float pts_out[][2],int *num);
void track_rightline(float pts_in[][2],int start,int end,float pts_out[][2],int *num);
void Find_Line(void);
void find_corners(void);
void Track_Choose(void);
float MidLine_Ctrl(void);
void banmaxian_center_scan_stop(void);
void Check_Loss(void);
uint8 Long_White(void);
//速度控制
void Speed_Change(void);
//最终集成
int Img_Copy(void);
void Img_deal(void);
//初始化
void Image_Init(void);
int Img_Copy(void);
//可视化
void view(void);

//目标板
int mid_line_detect(void);

void target_detect_deal(void);
#endif // VISION_H