/*
* vision.cpp
* vision part of the project
* created by lyc on 2026/03/20
* Copyright (c) 2026 by lyc, All Rights Reserved.
*/

// 必要的标准库头文件
#include <vector>
#include <math.h>
#include <string>
#include <iostream>   // 用于cout/cerr输出
#include <cstdint>    // 用于uint8_t
#include <chrono>     // 用于帧率控制
#include <thread>     // 用于延时
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/mat.hpp> // 显式包含cv::Mat头文件（避免未定义）

#include "inc.h"
#include "Rot.h"
#include "Ramp.hpp"
#include "Drive.hpp"
#include "Image.h"
#include "Circle.h"
#include "Cross.h"

// 在 PC 上调试时，手动定义算法需要的类型，不要引用 inc.h
#include <cstdint>
typedef uint8_t  uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;
typedef uint16_t uint16_t; // 确保兼容性

// 避免全局using namespace std，减少命名冲突
using std::cout;
using std::cerr;
using std::endl;
using std::vector;
using std::string;

//所有的参数
/*
 * @brief 图像处理核心缓存数组
 * @note  __attribute__((aligned(4))): 强制 4 字节对齐，确保龙芯 LA264 核心通过 32 位总线
 * 访问图像数据时效率最高，同时满足 DMA 传输对内存地址对齐的硬件要求。
 * @param IMG_H: 图像高度（行数）
 * @param IMG_W: 图像宽度（列数）
*/
lq_camera_ex cam(IMG_W,IMG_H,90,LQ_CAMERA_0CPU_MJPG);
LQ_NCNN ncnn_net_;
cv::Mat gray_pic,raw_pic,ipm_pic;
uint8 Image_Used[IMG_H][IMG_W] __attribute__((aligned(4))) = {{0}};
//大津法相关参数
uint8 OSTU_threshold=0;         //大津法算出来的阈值
uint8 OSTU_threshold_Add=15;    //阈值偏差，过滤背景噪声

//巡线相关参数
int Best_X=160;          //最长白列的X坐标(巡线起始点)
int Max_White_Height=0; //最长白列连续高度
int Standard_Width=240; //赛道在图像底部的像素宽度(用于补线)
int Block_Size=9;       //局部自适应窗口大小
int Clip_Value=4;       //经验值(通常4-10)，让阈值稍微比平均亮度黑一点
float img_k=0.999f;     //逆透视修正系数
uint8 starty=200;        //搜索的起始y坐标
uint8 Find_LineNum=20;  //寻找边界的最大行数限制，防止过度所搜导致的误识别，尤其是在环岛状态下
int Straight_TH=45;     //直线判断阈值，单位为像素，当拟合的直线与实际边界点的平均距离小于该值，则认为边界是直线
float conf_MAXL=0;      //左边线角点置信度，单位为0-1，越大表示越有可能是角点
float conf_MAXR=0;      //右边线角点置信度，单位为0-1，越大表示越有可能是角点
int ANGLE_DIS=20;       //角点之间的最小距离，单位为像素，防止过于密集的角点导致误识别
int Base_see=27;        //
int Tsee=27;            //
int Blue_Size=5;        //蓝色路径边界点数量，单位为点
uint8 CBHBUM=0;         //丢线保护标志，0表示不丢线保护，1表示丢线保护中
uint8 Base_starty=226;   //丢线保护起始行，单位为像素，通常设置在巡线起始点附近，例如95-110之间，根据实际情况调整
uint8 Check_Stop_TH=30; //丢线保护阈值，单位为像素，当连续丢线行数超过该值时，触发丢线保护机制，例如15-30之间，根据实际情况调整
uint8 Stop_CBH_TH=45;   //丢线保护的CBH阈值，单位为0-255，当CBH值超过该阈值时，认为当前行丢线，例如40-60之间，根据实际情况调整
uint8 Stop_car=0;       //丢线保护触发后是否停车，0表示不停车，1表示停车，根据实际需求设置
int Long_WhiteLine_X=0; //最长白线的X坐标，单位为像素，根据实际情况调整，例如设置在图像底部中央区域的范围内（50-110之间）
int Long_WhiteLine_Y=0; //最长白线的Y坐标，单位为像素，根据实际情况调整，例如设置在图像底部附近（90-110之间）
uint8 Long_WhiteLine=0; //
uint8 LossTH=40;        //丢线保护的丢线行数阈值，单位为行，当连续丢线行数超过该值时，触发丢线保护机制，例如20-40之间，根据实际情况调整
uint8 Car_go=0;         //
uint8 SpeedChangeH=1;//0:禁用  1：速度决策  2：直道速度决策 3:速度决策（无直道）
uint8 LongWhite_Speed=90; //
float straight_SpeedK=0.05f;
float WhiteLine_SpeedK=0.05f;
float dist=2.0f;           //
//图像识别相关

uint8 size=96;                    //图像大小96*96
bool model_initialized_=false;     //模型初始化标志位

//边界与中线数据
//使用int储存像素坐标更节省内存[0-159]
int Left_Edge[MAX_POINTS][2];     //左边界坐标
int Right_Edge[MAX_POINTS][2];      //右边界坐标

float (*Center_Line)[2];     //中心线数组
int *Center_Line_Num;        

float Blue_Left_Line[MAX_POINTS][2];    // 原始蓝色路径左边界坐标数组 [点索引][0=X坐标,1=Y坐标]
float Blue_Right_Line[MAX_POINTS][2];   // 原始蓝色路径右边界坐标数组 [点索引][0=X坐标,1=Y坐标]

float Left_Mid_Line[MAX_POINTS][2];  //左边中线坐标数组
int L_Mid_Num;    //左边中线点数量
float Right_Mid_Line[MAX_POINTS][2]; //右边中线坐标数组
int R_Mid_Num;    //右边中线点数量

float Left_Angel[MAX_POINTS];//左边界角点坐标数组，单位为像素坐标
float Right_Angel[MAX_POINTS];//右边界角点坐标数组，同理

float Nms_Left_Angel[MAX_POINTS];     //左边界角点置信度数组，单位为0-1，越大表示越有可能是角点
float Nms_Right_Angel[MAX_POINTS];    //右边界角点置信度数组，同上
// 路径边界备用坐标数组（用于拐角识别/滤波，避免修改原始坐标）
float Rep_Left_Line[MAX_POINTS][2];     // 左侧路径边界备用坐标数组 [点索引][0=X坐标,1=Y坐标]
int Rep_Lnum;              // RepLeft_Line数组中有效坐标点的数量（初始值为最大点数，后续更新为实际有效数）
float Rep_Right_Line[MAX_POINTS][2];    // 右侧路径边界备用坐标数组 [点索引][0=X坐标,1=Y坐标]
int Rep_Rnum;              // RepRight_Line数组中有效坐标点的数量（初始值为最大点数，后续更新为实际有效数）
//
float Left_Line[MAX_POINTS][2];
int Left_Num=MAX_POINTS;
float Right_Line[MAX_POINTS][2];
int Right_Num=MAX_POINTS;


float (*mid_pts)[2];  //指向当前使用的中线坐标数组的指针，根据左右边线的情况进行赋值
int *MidnumPts;   //指向当前使用的中线点数量的指针，根据左右边线的情况进行赋值
float Mid_Line[MAX_POINTS][2];   //当前使用的中线坐标数组，根据左右边线的情况进行赋值
int MidNum;
//环岛相关参数
uint8 LRob=0;
uint8 RRob=0;

//标志位
bool Left_Lost_Flag=false;     //左边线丢失标志
bool Right_Lost_Flag=false;    //右边线丢失标志
bool All_Lost_Flag=false;      //全丢标志(丢线保护)

int left_lost_flag[IMG_H];    //按行进行丢线标识
int right_lost_flag[IMG_H];   //丢线置1，不丢线置0

bool is_straightL=0;         //左边线是否为直线标志位
bool is_straightR=0;         //右边线是否为直线标志位

uint8 All_Check=1;            //全局扫描开关，确定当前图像不是乱码、没有丢线、具备基本的赛道形状时，置为1
uint8 check_flag=0;           //扫描标志位，正在扫描时置1，扫描结束后置0
bool Loss_Line=0;             //
//边界丢线数
int right_lost_times;
int left_lost_times;
int both_lost_times;  //双丢线数
//常规边界起始点
int boundry_start_left;
int boundry_start_right;
//环岛状态
int right_island_flag=0;  //1表示检测到右环岛，0表示无
int left_island_flag=0;   //1表示检测到左环岛

extern float base_speed;
extern int run;
extern std::atomic<float> camera_error;

LP_t LeftP={L_NULL,0};     //初始化左边道路的判断
LP_t RightP={L_NULL,0};    //初始化右边道路的判断
Mid_LineType LRType=MID_Left;   //初始化中线的状态
/*
island_state的取值含义：
0：正常巡线(normal)
1：预备入环(found tip) - 看到环岛的那个切点(外拐点)
2：正在入环(entering) - 正在斜着切进环岛
3：环岛内(inside) - 全速在环内绕圈，此时要强制单边巡线
4：准备出环(exiting) - 看到出环的内拐点
5：正在出环(leaving) - 走出环岛，回到直到
*/
//八邻域巡线方向
const int dir_front[4][2]={{0,-1},
                           {1,0},
                           {0,1},
                           {-1,0}};
const int dir_frontleft[4][2] = {
    {-1, -1}, // dir=0(上) 的左前方是 左上
    { 1, -1}, // dir=1(右) 的左前方是 右上
    { 1,  1}, // dir=2(下) 的左前方是 右下
    {-1,  1}  // dir=3(左) 的左前方是 左下
};
const int dir_frontright[4][2] = {
    { 1, -1}, // dir=0(上) 的右前方是 右上
    { 1,  1}, // dir=1(右) 的右前方是 右下
    {-1,  1}, // dir=2(下) 的右前方是 左下
    {-1, -1}  // dir=3(左) 的右前方是 左上
};

// 避障决策变量
float Dynamic_Mid_Offset = MID_DIS; 

extern float error_offset; // 用于存储当前的误差补偿值


//辅助函数
//计算CBH值，a和b分别是左右边界与中心线的距离，返回值越大表示
//偏离越严重
uint16 CBH(uint8 a,uint8 b)
{
    if(0==a+b)
        return 0;
    uint16 CHA=a>b?a-b:b-a;
    uint16 HE=a+b+1;
    CHA<<=8;
    return CHA/HE;
}

//计算拟合残差，pts是点坐标数组，n是点数量，返回值为平均残差平方和，
//单位为像素的平方
float residualSum(float pts[][2],int n)
{
    if(n<4)
        return -1;
    double sum_x=0.0f,sum_y=0.0f,sum_xy=0.0f,sum_xx=0.0f;
    double a=0,b=0;
    //计算求和值
    for(int i=0;i<n;i++){
        sum_y+=pts[i][0];
        sum_x+=pts[i][1];
        sum_xy+=pts[i][0]*pts[i][1];
        sum_xx+=pts[i][1]*pts[i][1];
    }
    double denominator=(n*sum_xx-sum_x*sum_x);
    if(fabs(denominator)<1e-6)
        return 999.0f;     //返回一个极大残差表示不可拟合

    //计算斜率a和截距b
    a=(n*sum_xy-sum_x*sum_y)/denominator;
    b=(sum_y-a*sum_x)/n;
    
    //计算残差平方和
    double sum_residual=0.0f;
    for(int i=0;i<n;i++){
        double x_pred=a*pts[i][1]+b;
        double residual=pts[i][0]-x_pred;
        sum_residual+=residual*residual;  //残差平方和
    }
    sum_residual/=n; //平均残差平方和
    //sum_residual/=n;
    return (float)sum_residual;
}

void get_highest(int*LWX,int*LWY)
{
    int lwline=160;
    int lw=230;
    for(int i=70;i<250;i+=4){
        if(AT(i,160)>=OSTU_threshold){
            for(int j=230;j>=10;--j){
                if((AT(i,j)>=OSTU_threshold&&AT(i,j-2)<OSTU_threshold&&AT(i,j-4)<OSTU_threshold)||j<=10){
                    if(j<lw){
                        lw=j;
                        lwline=i;
                    }
                    break;
                }
            }
        }
    }
    *LWX=lwline;
    *LWY=lw;
}

float fast_sqrt(float num)  //快速开根号，返回值为float型
{
    union{      //使用union结构而非struct结构，因为union共用同一片
        int i;  //内存，能够进行位移操作
        float x;
    } u;
    u.x=num;
    u.i=0x1fc00000+(u.i>>1);
    u.x=0.5f*(u.x+num/u.x);
    u.x=0.5f*(u.x+num/u.x);
    return u.x;
}

float fast_invsqrt(float num)//Quake III 快速平方根倒数
{
    union{        //类似于fast_sqrt
        int i;
        float x;
    }u;
    u.x=num;
    u.i=0x5f3759df-(u.i>>1);
    u.x=u.x*(1.5f-(0.5f*num*u.x*u.x));
    u.x=u.x*(1.5f-(0.5f*num*u.x*u.x));
    return u.x;
}


//点集三角滤波
void blur_points(float pts_in[][2],int num,float pts_out[][2])
{
    lq_assert(Blue_Size&1);    //奇数大小检验，只有奇数窗口才有对称的中心点
    int half=Blue_Size>>1;     //除以2
    for(int i=0;i<num;i++){    //遍历每一个点
        pts_out[i][0]=pts_out[i][1]=0;   //每次处理新的点之前，先清空累加器，准备接收新的加权值
        for(int j=-half;j<=half;j++){   //窗口，不只看当前点i，还看前后的邻居，向左、右各看half个点，
            pts_out[i][0]+=pts_in[clip(i+j,0,num-1)][0]*(half+1-abs(j));  //clip越界保护，边缘点会反复采样边界点来填充窗口
            pts_out[i][1]+=pts_in[clip(i+j,0,num-1)][1]*(half+1-abs(j));  //三角加权*(half+1-abs(j)，离得近的话权重高，离得远的话权重低，过滤突发尖锐噪点，同时不至于让整条线过度模糊
        }
        pts_out[i][0]/=(2*half+2)*(half+1)/2;    //归一化，变回真实坐标
        pts_out[i][1]/=(2*half+2)*(half+1)/2;    //内层循环当中不断累积，需要除以窗口的大小
    }
}

//线性插值法
//pts_out数组存储输出点
void resample_points(float pts_in[][2],int num1,float pts_out[][2],int*num2)
{
    if(num1<2){      //少于两个点无法进行插值，直接返回空结果
        *num2=0;
        return;
    }
    pts_out[0][0]=pts_in[0][0];      //初始化输出，将第一个输入点作为输出序列的
    pts_out[0][1]=pts_in[0][1];      //第一个点，len记录当前输出点的数量
    int len=1;
    for(int i=0;i<num1-1&&len<MAX_POINTS;i++){   //遍历每一段输入线段
        float x0=pts_in[i][0];   //相邻点(x0,y0)和(x1,y1)
        float y0=pts_in[i][1];
        float x1=pts_in[i+1][0];
        float y1=pts_in[i+1][1];
        do{    //插值核心
            float x=pts_out[len-1][0];  //上一个输出点
            float y=pts_out[len-1][1];

            float dx0=x0-x;     //最后一个点到线段起点(x0,y0)的坐标差
            float dy0=y0-y;
            float dx1=x1-x;     //最后一个点到线段终点(x1,y1)的坐标差
            float dy1=y1-y;

            float dist0=fast_sqrt(dx0*dx0+dy0*dy0);//上一个输出点到当前线段起点的距离
            float dist1=fast_sqrt(dx1*dx1+dy1*dy1);//上一个输出点到当前线段终点的距离

            float r0=(dist1-dist)/(dist1-dist0);//相似三角形系数比例，
            float r1=1-r0;               //算出来新点N在AB线段上的位置比例

            if(r0<0||r1<0)      //如果算出来N点在线段外面，说明已经过插值，退出循环
                break;
                             //标准线性插值公式
            x0=x0*r0+x1*r1;  //新X = 起点X * r0 + 终点X * r1
            y0=y0*r0+y1*r1;  //新Y = 起点Y * r0 + 终点Y * r1
            pts_out[len][0]=x0;   //保存新点
            pts_out[len][1]=y0;
            len++;
        }while(len<MAX_POINTS);
    }
    *num2=len;
}


//求角度，曲率角计算
void local_angel_points(float pts_in[][2],int num,float angle_out[])
{
    if(0==num)
        return;
    for(int i=0;i<num;i++){
        if(i<=0||i>=num-1){   //边界点直接给0，没办法算角度
            angle_out[i]=0;
            continue;
        }
        //计算前方向，后面的点指向现在的点
        float dx1=pts_in[i][0]-pts_in[clip(i-ANGLE_DIS,0,num-1)][0];
        float dy1=pts_in[i][1]-pts_in[clip(i-ANGLE_DIS,0,num-1)][1];
        //归一因子
        float dn1=fast_invsqrt(dx1*dx1+dy1*dy1);
        //计算后方向，现在的点指向前面的点
        float dx2=pts_in[clip(i+ANGLE_DIS,0,num-1)][0]-pts_in[i][0];
        float dy2=pts_in[clip(i+ANGLE_DIS,0,num-1)][1]-pts_in[i][1];
        //归一因子
        float dn2=fast_invsqrt(dx2*dx2+dy2*dy2);

        float c1=dx1*dn1;//前方向x
        float s1=dy1*dn1;//前方向y
        float c2=dx2*dn2;//后方向x
        float s2=dy2*dn2;//后方向y
                              //向量叉乘  向量点乘 
                              //左/右     转的多/少  1-直走，0-90°，-1-180°
        angle_out[i]=atan2f(c1*s2-c2*s1,c2*c1+s2*s1);
                     //算出两个方向之间的夹角
    }
}

//角度变化率非极大抑制
//把假弯道，抖动角度全都清零
//一段弯道当中只保留最弯的那一个点，其他全变成0，防止抖动和误判
void nms_angle(float angle_in[],int num,float angle_out[])
{
    if(0==num)
        return;
    for(int i=0;i<num;i++){
        angle_out[i]=angle_in[i];
        for(int j=-ANGLE_DIS;j<=ANGLE_DIS;j++){
            if(fabs(angle_in[clip(i+j,0,num-1)])>fabs(angle_out[i])){
                angle_out[i]=0;
                break;
            }
        }
    }
}

/*
大津法进行阈值的计算
*/
void get_OSTU(void)
{    
    static long long frame_counter=0;
    static uint8_t last_threshold=130;
    frame_counter++;
    if(frame_counter%2){
        OSTU_threshold=last_threshold;
        return;
    }

    #define GrayScale  (256)
    #define END_Y      (4*IMG_H/5)
    #define START_Y    (1*IMG_H/5)
    int pixel_Count[GrayScale]={0};   //每个灰度值所占的像素个数
    float pixel_Proportion[GrayScale]={0};   //每个灰度值所占的总像素的比例
    uint8 threshold=130;     //最终的阈值threshold
    int total_pixels=((END_Y-START_Y)>>1)*((IMG_W-60)>>1);    //计数总的pixel数量
    int i,j;

    for(j=START_Y;j<END_Y;j+=2){
        for(i=30;i<IMG_W-30;i+=2){
            pixel_Count[(int)AT(i,j)]++;
        }
    }
    float u=0;
    for(i=0;i<GrayScale;i++){
        if(pixel_Count[i]==0)
            continue;
        pixel_Proportion[i]=(float)pixel_Count[i]/total_pixels;   //计算每个像素在整幅图像当中的比例
        u+=i*pixel_Proportion[i];    //总平均灰度
    }
    float max_variance = -1.0f;
    float w0 = 0.0f;         // 背景比例
    float w1 = 0.0f;         // 前景比例
    float sum0 = 0.0f;       // 背景灰度总贡献
    float sum1 = u;          // 前景灰度总贡献，初始为全图总均值 u
    for(i=0;i<GrayScale;i++){     //每一次循环都是一次完整类间方差计算(两个for叠加为1个)
        w0 += pixel_Proportion[i];
        w1 = 1.0f - w0;
        // 边界检查：如果背景或前景为空，跳过本次循环
        if (w0 <= 0.0001f || w1 <= 0.0001f) 
            continue;

        // 更新灰度贡献（注意：i * 比例 = 该灰度级对全图均值的贡献）
        float delta_sum = (float)i * pixel_Proportion[i];
        sum0 += delta_sum;
        sum1 -= delta_sum;
        // 计算平均灰度 mu0 和 mu1
        float mu0 = sum0 / w0;
        float mu1 = sum1 / w1;

        float diff = mu0 - mu1;  //公式：σ^2=w0*w1*(u0-u1)^2
        float variance = w0 * w1 * diff * diff;

        if (variance > max_variance) {  //最大值进行更新
            max_variance = variance;
            threshold = (uint8)i;
        }
    }
    last_threshold=threshold+OSTU_threshold_Add;
    OSTU_threshold=(uint8)last_threshold;
    //cout<<(int)OSTU_threshold<<endl;
}

//局部亮度综合的滑动窗口更新函数
/**
 * @brief  局部亮度总和的滑动窗口更新函数
 * @param  sum   当前(x, y)位置下，Block_Size * Block_Size 区域的像素总和
 * @param  x     移动前的横坐标
 * @param  y     移动前的纵坐标
 * @param  half  窗口半径 (half = Block_Size / 2)
 * @param  dirx  x方向的移动偏移量 (来自 dir_front，取值 -1, 0, 1)
 * @param  diry  y方向的移动偏移量 (来自 dir_front，取值 -1, 0, 1)
 * @return 更新后的新位置下的像素总和
 * * @note   原理：
 * 当爬虫从 A 点移动到 B 点时，大部分覆盖的窗口区域是重叠的。
 * 新窗口总和 = 旧窗口总和 - 离开窗口的那一排像素 + 进入窗口的那一排像素。
 */
static inline int updata_sum(int sum,int x,int y,int half,int dirx,int diry)
{
    uint8 flag=0;
    if(diry!=0){     //diry!=0&&Check_Password()
        if(diry>0){
            for(int dx=-half;dx<=half;++dx){
                sum-=AT(x+dx,y-half);
                sum+=AT(x+dx,y+half+1);
            } 
        }
        else{
            for(int dx=-half;dx<=half;++dx){
                sum-=AT(x+dx,y+half);
                sum+=AT(x+dx,y-half-1);
            }
        }
        flag=1;
    }
    if(dirx!=0){     //dirx!=0&&Check_Point
        if(flag)
            y+=diry;
        if(dirx>0){
            for(int dy=-half;dy<=half;dy++){
                sum-=AT(x-half,y+dy);
                sum+=AT(x+1+half,y+dy);
            }
        }
        else{
            for(int dy=-half;dy<=half;++dy){
                sum-=AT(x+half,y+dy);
                sum+=AT(x-1-half,y+dy);
            }
        }
    }
    return sum;
}
//=========================================================================================
//  八邻域爬线
//左手巡线逆透视
/**
 * @brief  基于局部自适应阈值的左手边界跟踪算法（带逆透视映射）
 * @param  int x  搜索起始点的横坐标（通常由底行扫描确定）
 * @param  int y  搜索起始点的纵坐标（通常是靠近图像底部的行）
 * @param  float pts [out] 存储爬行结果的二维数组，pts[n][0]为物理X, pts[n][1]为物理Y
 * @param  int num [out] 实际搜索到的有效点数（步数）
 * * @note   算法原理：
 * 1. 以(start_x, start_y)为种子点，始终保持黑线在左侧，白赛道在前方/右前方爬行。
 * 2. 动态计算当前坐标周围 BLOCK_SIZE 区域的平均灰度作为局部阈值。
 * 3. 每一跳都会通过映射表/函数将像素坐标转换为车体坐标系下的物理坐标（mm）。
 * @warning 调用前需确保起点周围不越界，且输入图像 Image_Used 已完成预处理。
 */
#define BLOCK_AREA (Block_Size*Block_Size)
#define INV_AREA ((1<<16)/BLOCK_AREA)
void findline_lefthand_adaptive(int x,int y,float pts[][2],int *num)
{
    //lq_assert(!Check_PassWord());
    lq_assert(num&&*num>=0);
    lq_assert(Block_Size>1&&(Block_Size&1));
    int half=Block_Size>>1;    //视野半径，距离图像边界至少要这个值
    //step:当前已经成功识别并记录的赛道边缘点的数量
    //dir:爬虫当前的“行进车头方向”
    //turn:当前像素点坐标不变的情况下，尝试改变dir的次数，决定是否退出循环
    int step=0,dir=0,turn=0;
    int local_thres=0;   //自适应均值阈值
    int local_sum=0;     //当前爬虫脚下的亮度总和
    for(int dy=-half;dy<=half;dy++){  //求得亮度总和
        for(int dx=-half;dx<=half;dx++){
            local_sum+=AT(x+dx,y+dy);
        }
    }
    local_thres=((local_sum*INV_AREA)>>16)-Clip_Value;   //求得平均灰度
    while(step<MAX_POINTS&&half+1<x&&x<IMG_W-half-1&&half+1<y&&y<IMG_H-half-1&&turn<4){
        //对于当前位置的正前方
        int front_value=AT(x+dir_front[dir][0],y+dir_front[dir][1]);
        //对于当前位置的正前方的左手第一位
        int frontleft_value=AT(x+dir_frontleft[dir][0],y+dir_frontleft[dir][1]);
        if(front_value<local_thres){  //如果front_value比这里的阈值更黑，说明撞墙了
            dir=(++dir)&3;
            turn++;
        }
        else if(frontleft_value<local_thres){
            local_sum=updata_sum(local_sum,x,y,half,dir_front[dir][0],dir_front[dir][1]);            
            x+=dir_front[dir][0];
            y+=dir_front[dir][1];
            pts[step][0]=inv_mapx[y][x];
            pts[step][1]=inv_mapy[y][x];
            //Right_Edge[step][0]=x;
            //Right_Edge[step][1]=y;
            step++;
            turn=0;
            //local_thres=local_sum/(Block_Size*Block_Size)-Clip_Value;
            local_thres=((local_sum*INV_AREA)>>16)-Clip_Value;
        }
        else{
            local_sum=updata_sum(local_sum,x,y,half,dir_frontleft[dir][0],dir_frontleft[dir][1]);
            x+=dir_frontleft[dir][0];
            y+=dir_frontleft[dir][1];
            dir=(dir+3)&3;
            pts[step][0]=inv_mapx[y][x];
            pts[step][1]=inv_mapy[y][x];
            //Right_Edge[step][0]=x;
            //Right_Edge[step][1]=y;
            step++;
            turn=0;
            //local_thres=local_sum/(Block_Size*Block_Size)-Clip_Value;
            local_thres=((local_sum*INV_AREA)>>16)-Clip_Value;
        }
    }    
    *num=step;
}

//右手巡线逆透视
/**
 * @brief  基于局部自适应阈值的右手边界跟踪算法（带逆透视映射）
 * @param  int x  搜索起始点的横坐标（通常由底行扫描确定）
 * @param  int y  搜索起始点的纵坐标（通常是靠近图像底部的行）
 * @param  float pts [out] 存储爬行结果的二维数组，pts[n][0]为物理X, pts[n][1]为物理Y
 * @param  int num [out] 实际搜索到的有效点数（步数）
 * * @note   算法原理：
 * 1. 以(start_x, start_y)为种子点，始终保持黑线在右侧，白赛道在前方/右前方爬行。
 * 2. 动态计算当前坐标周围 BLOCK_SIZE 区域的平均灰度作为局部阈值。
 * 3. 每一跳都会通过映射表/函数将像素坐标转换为车体坐标系下的物理坐标（mm）。
 * @warning 调用前需确保起点周围不越界，且输入图像 Image_Used 已完成预处理。
 */
void findline_righthand_adaptive(int x,int y,float pts[][2],int *num)
{
    //lq_assert(!Check_PassWord());
    lq_assert(num&&*num>=0);
    lq_assert(Block_Size>1&&(Block_Size&1)); //局部自适应窗口必须大于1并且为奇数(确保中心唯一)
    //一下变量作用同findline_lefthand_adaptive中的变量
    int half=Block_Size>>1;
    int step=0,dir=0,turn=0;
    int local_thres=0;
    int local_sum=0;

    for(int dy=-half;dy<=half;dy++){
        for(int dx=-half;dx<=half;dx++){
            local_sum+=AT(x+dx,y+dy);
        }
    }
    local_thres=((local_sum*INV_AREA)>>16)-Clip_Value;   //求得平均灰度
    while(step<MAX_POINTS&&half<x&&x<IMG_W-half&&half<y&&y<IMG_H-half&&turn<4){
        int front_value=AT(x+dir_front[dir][0],y+dir_front[dir][1]);
        int frontright_value=AT(x+dir_frontright[dir][0],y+dir_frontright[dir][1]);
        if(front_value<local_thres){
            dir=(dir+3)&3;
            turn++;
        }
        else if(frontright_value<local_thres){
            local_sum=updata_sum(local_sum,x,y,half,dir_front[dir][0],dir_front[dir][1]);
            x+=dir_front[dir][0];
            y+=dir_front[dir][1];
            pts[step][0]=inv_mapx[y][x];
            pts[step][1]=inv_mapy[y][x];
            //Right_Edge[step][0]=x;
            //Right_Edge[step][1]=y;
            step++;
            turn=0;
            //local_thres=local_sum/(Block_Size*Block_Size)-Clip_Value;
            local_thres=((local_sum*INV_AREA)>>16)-Clip_Value;
        }
        else{
            local_sum=updata_sum(local_sum,x,y,half,dir_frontright[dir][0],dir_frontright[dir][1]);
            x+=dir_frontright[dir][0];
            y+=dir_frontright[dir][1];
            dir=(++dir)&3;
            pts[step][0]=inv_mapx[y][x];
            pts[step][1]=inv_mapy[y][x];
            //Right_Edge[step][0]=x;
            //Right_Edge[step][1]=y;
            step++;
            turn=0;
            //local_thres=local_sum/(Block_Size*Block_Size)-Clip_Value;
            local_thres=((local_sum*INV_AREA)>>16)-Clip_Value;
        }
    }
    *num=step;
}
//=====================================================================================================
// 中线偏移
//左边线跟踪中线
void track_leftline(float pts_in[][2],int start,int end,float pts_out[][2],int *num)
{
    *num=end-start;   //计算输出点的数量
    //安全检查，防止索引越界或者非法输入
    if(start>end||end<=0||start<0||start>=MAX_POINTS)
        return;

    //遍历每一个边线点，计算它对应的中线点
    for(int i=start;i<end;i++){
        float dx=pts_in[clip(i+APPROX_NUM,0,end-1)][0]-pts_in[clip(i-APPROX_NUM,0,end-1)][0];
        float dy=pts_in[clip(i+APPROX_NUM,0,end-1)][1]-pts_in[clip(i-APPROX_NUM,0,end-1)][1];
        float dn=fast_invsqrt(dx*dx+dy*dy);
        dx*=dn;
        dy*=dn;
        pts_out[i-start][0]=pts_in[i][0]-dy*Dynamic_Mid_Offset*img_k;
        pts_out[i-start][1]=pts_in[i][1]+dx*Dynamic_Mid_Offset*img_k;
    }
}


//右边线跟踪中线
void track_rightline(float pts_in[][2],int start,int end,float pts_out[][2],int *num)
{
    *num=end-start;   //计算输出点的数量
    //安全检查，防止索引越界或者非法输入
    if(start>end||end<=0||start<0||start>=MAX_POINTS)
        return;

    //遍历每一个边线点，计算它对应的中线点
    for(int i=start;i<end;i++){
        float dx=pts_in[clip(i+APPROX_NUM,0,end-1)][0]-pts_in[clip(i-APPROX_NUM,0,end-1)][0];
        float dy=pts_in[clip(i+APPROX_NUM,0,end-1)][1]-pts_in[clip(i-APPROX_NUM,0,end-1)][1];
        float dn=fast_invsqrt(dx*dx+dy*dy);
        dx*=dn;
        dy*=dn;
        pts_out[i-start][0]=pts_in[i][0]+dy*Dynamic_Mid_Offset*img_k;
        pts_out[i-start][1]=pts_in[i][1]-dx*Dynamic_Mid_Offset*img_k;
        //pts_out[i-start][0]=(Left_Line[i][0]+Right_Line[i][0])/2.0;
        //pts_out[i-start][1]=Right_Line[i][1];
    }
}
//=======================================================================================================================
//双边巡线逻辑
void Find_Line(void)
{
    //设置为-1作为状态标记，以防底部没找到右边缘，直接转入丢线补救逻辑
    int Lx,Ly=-1,Rx,Ry=-1;    //Lx，Ly是左边线爬虫的起点坐标;Rx,Ry是右边线爬虫的起点坐标
    int Start=starty;     //搜索的起始行坐标
    int L_find_num,R_find_num;  //纵向深度限制(步长限制),代表最多向上找多少行来确定种子点
    if(Circle_LOut==circle_type)   //如果是环岛，优先使用右边线爬虫，寻找右边线的起点
        R_find_num=clip(Find_LineNum+80.0,0,Start);  //右环寻找点数限制
    else   //非环岛，优先使用左边线爬虫，寻找左边先的起点
        R_find_num=Find_LineNum;  //左环寻找点数限制

    if(Circle_ROut==circle_type)   //右环，先找到右边界起点
        L_find_num=clip(Find_LineNum+80.0,0,Start);//左边线寻找点数限制，右环时适当地增加寻找范围，防止起点过于靠近图像底部导致丢线
    else   //非右环，正常寻找左边线起点
        L_find_num=Find_LineNum;

    for(int i=0;i<R_find_num;++i){  //从起始行开始向前20行搜索右边线的起点
        if(AT(IMG_W>>1,Start-i)<OSTU_threshold)
            continue;   //如果第一个点就是黑点，跳过本次搜索
        for(Rx=(IMG_W>>1);Rx<IMG_W-Block_Size;Rx++){  //从中间往右搜索黑白跳变
            if(AT(Rx,Start-i)>OSTU_threshold&&AT(Rx+1,Start-i)<OSTU_threshold){
                Ry=Start-i;  //保存跳变点，跳出循环
                break;
            }
        }
        if(Ry!=-1)
            break;  //如果找到了跳变点就跳出循环
    }
    
    if(Ry!=-1)  //已经找到了右边线的起点，寻找右边线
        findline_righthand_adaptive(Rx,Ry,Right_Line,&Right_Num);
    else{   //如果没有找到跳变点，判断右下色块颜色
        int temp=0;
        for(int i=1;i<=10;i++){
            for(int j=1;j<=30;j++){
                temp += AT(IMG_W - i - 10, (IMG_H - 10) - j);
            }
        }
        temp/=10*30;      //计算平均灰度
        uint8 Rf=0;    //根据平均灰度判断右下角色块颜色，进而判断是右环还是左环
        Ry=Start;   //如果没有找到右边线起点，就把起点设置在图像底部，进入丢线补救逻辑
        Rx=IMG_W-(Block_Size>>1);   //如果是右环，起点设置在图像右侧边界附近
        do{
            if(temp>OSTU_threshold){
                //cout<<"右下角是白色，判断为右环，进入右环寻找逻辑"<<endl;
                break;   //如果右下是白色，说明为右急弯，不寻找右线//非出圆环
            }
            for(Rx=IMG_W-(Block_Size>>1);Rx>(2*IMG_W/5);--Rx){
                if(AT(Rx,Ry)>OSTU_threshold)
                    break;  //如果右下角为黑色，说明存在丢线，寻找起始行最右边的第一个白色点
            }
            for(int i=0;i<80;i++){ //从起始行最右边第一个白色点开始向上找黑白跳变点
                if(AT(Rx,Ry-i)>OSTU_threshold&&AT(Rx,Ry-i-1)<OSTU_threshold){
                    Ry-=i;     //保存跳变点，跳出循环
                    Rf=1;      //如果找到了跳变点，说明存在右边线，设置右环标志
                    break;
                }
            }
        }while(0);
        if(Rf)   //如果右环标志被设置，说明存在黑白跳变与右边线，寻找右边线
            findline_righthand_adaptive(Rx,Ry,Right_Line,&Right_Num);
        else 
            Right_Num=0;//丢线补线
    }
    //从起始行开始向前20行搜索左边先的起点
    for(int i=0;i<L_find_num;++i){
        if(AT(IMG_W>>1,Start-i)<OSTU_threshold)
            continue;  //如果第一个点就是黑点，跳过本次搜索
        //从中间向左寻找黑白跳变点
        for(Lx=(IMG_W>>1);Lx>Block_Size;Lx--){
            if(AT(Lx,Start-i)>OSTU_threshold&&AT(Lx-1,Start-i)<OSTU_threshold){
                Ly=Start-i;
                break;
            }
        }
        if(-1!=Ly)
            break;  //如果找到了跳变点就退出循环
    }
    if(-1!=Ly)//找到了跳变点，寻找左线
        findline_lefthand_adaptive(Lx,Ly,Left_Line,&Left_Num);
    else{   //如果没有找到跳变点，判断左下色块颜色
        int temp=0;
        for(int i=1;i<=10;++i){
            for(int j=1;j<=30;++j){
                temp+=AT(i+10,(IMG_H-10)-j);  //计算左下角区域的平均灰度
            }
        }
        temp/=10*30;  //计算平均灰度值
        uint8 Lf=0;
        Ly=Start;  //如果没有找到左边线起始点，就把起始点位置设置在图像底部
        Lx=Block_Size>>1;  //如果是左环，起点设置在图像左侧边界位置
        do{
            if(temp>OSTU_threshold)
                break;  //如果左下角是白色，说明为左急弯，不寻找右线，非出圆环
            for(Lx=Block_Size>>1;Lx<(3*IMG_W/5);++Lx){
                if(AT(Lx,Ly)>OSTU_threshold)
                    break;  
                //如果左下角为黑色，说明存在丢线，寻找起始行最左边的第一个白色点             
            }
            //从起始行最右边第一个白色点开始向上找黑白跳变点
            for(int i=0;i<80;i++){
                if(AT(Lx,Ly-i)>OSTU_threshold&&AT(Lx,Ly-i-1)<OSTU_threshold){
                    Ly-=i;  //保存跳变点，跳出循环
                    Lf=1;  //如果找到了跳变点，说明存在左边线，设置左环标志
                    break;
                }
            }
        }while(0);
        if(Lf)  //如果存在黑白跳变，寻找左线
            findline_lefthand_adaptive(Lx,Ly,Left_Line,&Left_Num);
        else
            Left_Num=0;  //丢线补线
    }

}
//=========================================================================
//寻找赛道边界的角点(环岛入口/出口)
void find_corners(void)
{
#define L_TH (43)    //左边线角点的阈值，单位为mm
    LeftP.LP=L_NULL,RightP.LP=L_NULL;   //初始化角点状态为未找到
    LeftP.ID=(uint8)Rep_Lnum,RightP.ID=(uint8)Rep_Rnum;
    //标志位，如果Rep_Lnum或Rep_Rnum超过阈值，说明找到了对应的角点
    is_straightL=(Rep_Lnum>Straight_TH);
    is_straightR=(Rep_Rnum>Straight_TH);
    //for_num是一个临时变量，用于存储Rep_Lnum和Rep_Rnum中较小的一个，
    //作为后续角点判断的参考值
    uint8 for_num=MIN(Rep_Lnum,60),i;
    conf_MAXL=conf_MAXR=0;   //初始化角点置信度为0
    for(i=0;i<for_num;++i){
        if(Nms_Left_Angel[i]==0)
            continue;
        //clip函数用于限制i-ANGLE_DIS和i+ANGLE_DIS的值在
        //0到Rep_Lnum-1的范围内，防止数组越界
        //im1和ip1分别是i的前一个和后一个索引，
        //代表了当前角度索引的左右邻居
        int im1=clip(i-ANGLE_DIS,0,Rep_Lnum-1);
        int ip1=clip(i+ANGLE_DIS,0,Rep_Lnum-1);
        //conf变量计算了当前角度索引i的置信度，基于非极大值抑制的思想
        float conf=fabs(Nms_Left_Angel[i])-(fabs(Nms_Left_Angel[im1])+fabs(Nms_Left_Angel[ip1]))/2;
        //对conf_MAX进行更新，获得当前最大的置信度，便于后续
        //判断角点是否可靠
        if(fabs(conf)>fabs(conf_MAXL))
            conf_MAXL=conf;
        if(conf>1.05&&conf<2.18f){  //65-115度之间的角点才是合理的
            if(Left_Angel[i]>0){   //如果角度是正的，说明是左边线的角点
                LeftP.LP=L_In;    //设置左边线角点状态为找到
                LeftP.ID=i;       //记录角点类型
            }
            else{
                LeftP.LP=L_Out;    //设置左边线角点状态为找到
                LeftP.ID=i;        //记录角点类型
            }
        }
        if(is_straightL&&conf>0.09960f) //
            is_straightL=0;  //如果在知道状态下找到了合理的角点，说明之前的直道状态是错误的，重置为未找到
        //如果在直道状态下找到了合理的角点，说明之前的直道状态是错误的
        //重置为未找到
        if(0==is_straightL&&(L_NULL!=LeftP.LP))
            break;  
    }

    //开始寻找右边线的角点，方法与左角点的寻找方法类似
    for_num=MIN(Rep_Rnum,60);
    for(i=0;i<for_num;++i){
        if(Nms_Right_Angel[i]==0)
            continue;
        int im1=clip(i-ANGLE_DIS,0,Rep_Rnum-1);
        int ip1=clip(i+ANGLE_DIS,0,Rep_Rnum-1);
        float conf=fabs(Nms_Right_Angel[i])-(fabs(Nms_Right_Angel[im1])+fabs(Nms_Right_Angel[ip1]))/2;
        if(fabs(conf)>fabs(conf_MAXR))
            conf_MAXR=conf;
        if(conf>1.05f&&conf<2.18f)  //65-115度之间的角点才是合理的
        {
            if(Right_Angel[i]<0){  //如果角度是负的，说明是右边线的角点
                RightP.LP=L_In;   //设置右边线角点状态为找到
                RightP.ID=i;      //记录角点类型
            }
            else{
                RightP.LP=L_Out;  //设置右边线角点状态为找到
                RightP.ID=i;      //记录角点类型
            }
        }
        if(is_straightR&&conf>0.09960f) //
            is_straightR=0;  //如果在知道状态下找到了合理的角点，说明之前的直道状态是错误的，重置为未找到
        if(0==is_straightR&&(L_NULL!=RightP.LP))
            break;
    }
    //cout<<"conf_MAXL:"<<conf_MAXL<<" conf_MAXR:"<<conf_MAXR<<endl;
    //最后根据找到的角点类型和置信程度，判断当前的行进状态
    LD=residualSum(Rep_Left_Line,Rep_Lnum);
    RD=residualSum(Rep_Right_Line,Rep_Rnum);
    // 1. 全局平直度纠错：利用最小二乘法残差(LD/RD)判断
    // 即便局部看似有角点(conf_MAX)，但如果整体残差极小(<=1.5)且长度足够，则强制判定为直道，防止噪点干扰
    if(conf_MAXL<0.600f&&LD<=1.5f&&Rep_Lnum>Straight_TH)
        is_straightL=1;
    if(conf_MAXR<0.600f&&RD<=1.5f&&Rep_Rnum>Straight_TH)
        is_straightR=1;
    //cout<<"LD: "<<LD<<"  RD: "<<RD<<"  Rep_Rnum: "<<Rep_Rnum<<"  Rep_Lnum: "<<Rep_Lnum<<endl; 
    // 2. 左右间距校验（针对左角点）：
    // 当右边线很直但左边发现角点时，计算该角点到右线的物理距离dn
    if(is_straightR&&L_NULL!=LeftP.LP){
        float dn=0;
        float dx=Rep_Left_Line[LeftP.ID][0]-Rep_Right_Line[clip(LeftP.ID,0,Rep_Rnum-1)][0];
        float dy=Rep_Left_Line[LeftP.ID][1]-Rep_Right_Line[clip(LeftP.ID,0,Rep_Rnum-1)][1];
        dn=fast_sqrt(dx*dx+dy*dy);
        //cout<<"Left corner distance to right line: "<<dn<<" mm"<<endl;
        if(dn<L_TH){
            LeftP.LP=L_NULL;
            LRob=1;
        }
        else
            LRob=0;
    }
    // 3. 左右间距校验（针对右角点）：
    // 同理，当左边线很直但右边发现角点时，校验其到左线的距离是否符合赛道物理宽度
    if(is_straightL&&L_NULL!=RightP.LP){
        float dn=0;
        float dx=Rep_Left_Line[clip(RightP.ID,0,Rep_Lnum-1)][0]-Rep_Right_Line[RightP.ID][0];
        float dy=Rep_Left_Line[clip(RightP.ID,0,Rep_Lnum-1)][1]-Rep_Right_Line[RightP.ID][1];
        dn=fast_sqrt(dx*dx+dy*dy);
        //cout<<"Right corner distance to left line:  "<<dn<<" mm"<<endl;
        if(dn<L_TH){
            RightP.LP=L_NULL;
            RRob=1;
        }
        else
            RRob=0;
    }
    //cout<<"LeftP: "<<(int)LeftP.LP<<" ID: "<<(int)LeftP.ID<<" RightP: "<<(int)RightP.LP<<" ID: "<<(int)RightP.ID;
    //cout<<"  is_straightL: "<<is_straightL<<" is_straightR: "<<is_straightR<<endl;
    //cout<<"LRob: "<<(int)LRob<<" RRob: "<<(int)RRob<<endl;
}

//=================================================================================
//根据左右边线的情况选择跟踪方式
/* --- 边线可靠性决策（领航员选择） --- */
/* 功能：根据左右边线有效点的数量(L_Mid_Num/R_Mid_Num)，
   判断当前哪一边的赛道信息更完整，从而决定中线合成的基准。
   
   决策逻辑：
   1. 数量优先：谁的点数多、更长，谁的权重就高。
   2. 丢线保护：若某侧点数极少（<20或不到对侧一半），判定为丢线，切换至对侧。
   3. 干扰干预：LRob/RRob 为真时，说明检测到了赛道外的干扰物，
      强制锁定信任方向，触发抗干扰补线逻辑，防止由于假角点导致的猛打方向。
*/
void Track_Choose(void)
{
    //如果左边点数不到右边的一半，且左边点数非常少：判定左边严重丢线或者遇到大弯/入环吧，切换为信任右边
    if (L_Mid_Num < R_Mid_Num / 2 && L_Mid_Num < 80)
        LRType=MID_Right;
    //如果右边点数不到左边的一半，且右边点数非常少：判定右边严重丢线，切换为信任左边
    else if (R_Mid_Num < L_Mid_Num / 2 && R_Mid_Num < 80)
        LRType=MID_Left;
    //极度丢线处理，如果左边点数已经少于60个(视野内几乎没线了):只要右边点数比左边多，
    //哪怕只多一点，也要赶紧切换到右边
    else if (L_Mid_Num < 60 && R_Mid_Num > L_Mid_Num)
        LRType=MID_Right;
    //对称处理，如果右边点数极少(少于30个)且左边更有优势，切换到信任左边
    else if (R_Mid_Num < 60 && L_Mid_Num > R_Mid_Num)
        LRType=MID_Left;
    //常规对比：当上述丢线情况都不存在时，简单粗暴的“哪边点多”
    //保证在正常行驶时，小车总是跟踪信息量更丰富的一侧
    else if (L_Mid_Num < R_Mid_Num)
        LRType=MID_Right;
    else
        LRType=MID_Left;
    
    //抗干扰强制修正
    //如果再find_corners中出发了LRob(左侧机器人/物体干扰)
    //强制锁定LRType为MID_Left
    //由于检测到左侧有干扰，需要进入特定的补线或者单边避障模型
    
    if(LRob){
        LRType=MID_Left;
        Dynamic_Mid_Offset=MID_DIS;
    }
    //如果触发了RRob(右侧机器人/物体干扰)
    //强制锁定LRType为MID_Right，配合后续算法绕过干扰
    if(RRob){
        LRType=MID_Right;
        Dynamic_Mid_Offset=MID_DIS;
    }
    /*
    if(winner==Image_Type::weapon)   
        LRType=MID_Left;
    else if(winner==Image_Type::supplies)
        LRType=MID_Right;*/
}

//==================================================================
/**
 * @brief 中线控制函数 (基于像素坐标系)
 * @return float 偏差值 error (像素单位，左负右正)
 */
/**
 * @brief 修正后的中线控制函数
 * @return float 修正后的偏差值
 */
float MidLine_Ctrl(void)
{
    static float last_err1 = 0.0f, last_err2 = 0.0f, last_err3 = 0.0f;
    
    // 在逆透视图(IPM)中，由于图像已经是对称拉伸的
    // cx 依然建议设为图像宽度的一半（160/2 = 80）
    float cx = mapx[235][160]+1.239945f-1.32755f; 
    float cy = mapy[235][160]+14.0f; // 底部

    // 角色绑定（保持不变）
    switch(LRType){
        case MID_Left:  
            mid_pts = Left_Mid_Line;  
            MidnumPts = &L_Mid_Num; 
            break;
        case MID_Right: 
            mid_pts = Right_Mid_Line; 
            MidnumPts = &R_Mid_Num; 
            break;
        case MID_Ramp:  
            Tsee = 10; 
            mid_pts = RampMID; 
            MidnumPts = &RampMIDNum; 
            break;
        default: 
            break;
    }

    // --- 寻找起始点 ---
    int begin_id = -1;
    float min_dist = 1e10;
    for(int i = 0; i < clip(*MidnumPts, 0, 100); i++){
        float dx = mid_pts[i][0] - cx;
        float dy = mid_pts[i][1] - cy;
        float d2 = dx*dx + dy*dy;
        if(d2 < min_dist){ 
            min_dist = d2; 
            begin_id = i; 
        }
    }

    if(begin_id != -1 && (*MidnumPts) - begin_id >= 5) {
        // 重采样：现在的 Mid_Line 点与点之间距离是固定的（比如 2 像素）
        resample_points((mid_pts + begin_id), (*MidnumPts) - begin_id, Mid_Line, &MidNum); 

        // --- 预瞄点 Tsee 的物理意义 ---
        // 在 IPM 图中，每一个像素代表的物理距离是相等的
        // Tsee = 30 就代表车头前方固定距离（例如 30cm）处的偏差
        int target_id = clip(Tsee, 2, MidNum - 3);
        
        // 空间平均（依然保留，增加稳定性）
        float sum_x = 0;
        float sum_y=0;
        for(int j = -2; j <= 2; j++) 
            sum_x += Mid_Line[target_id + j][0];
        float avg_x=sum_x/5.0f;
        
        // 计算当前偏差
        float current_error = avg_x-cx;

        // 时间滤波更新
        last_err3 = last_err2; 
        last_err2 = last_err1; 
        last_err1 = current_error;
    } 
    else {
        last_err1 *= 0.5f; // 丢线衰减
    }
    //cout<<last_err1 * 0.7f + last_err2 * 0.2f + last_err3 * 0.1f<<endl;
    return last_err1 * 0.7f + last_err2 * 0.2f + last_err3 * 0.1f+error_offset;
}

//=================================================================

void Check_Loss(void)
{
#define FPS_TH     (8)  //连续检测到有40帧有障碍物则停车
    All_Check=1;    //先默认允许正常巡线  1-一切正常  0-前方有障碍物，禁止正常巡线
    static uint16 fps=0;  //fps计数器
    static uint8 Loss_fps=0;  //丢帧持续计数器
    static uint8 Block_num=0;  //保障保护时间，防止反复抖动
    if(check_flag){  //如果为1，说明障碍消失以及丢线状态恢复，所有制初始化
        check_flag=0;
        fps=0;
        Loss_fps=0;
        Block_num=0;
    }

    if(fps<FPS_TH)  //每运行一帧，fps+1
        fps++;
    CBHBUM=0; //统计当前行有多少个黑白跳变
    uint8 temp_Num=0;//记录跳变最多的那一行

    for(int j=-10;j<=10;++j){   //把车头前面11行都扫描一遍
        CBHBUM=0;
        for(uint i=1;i<IMG_W-5;i+=4){  //图像底部这一行从左扫到右，每两个像素之间比较一下，是不是突然从白变黑或者黑变白
                        //如果变化剧烈，说明前面越有可能有车，有障碍物
            if(CBH(AT(i,Base_starty-j),AT(i+4,Base_starty-j))>Stop_CBH_TH)
                CBHBUM++;  //记录这一行有多少个剧烈变化点
        }
        if(CBHBUM>temp_Num)   //前方障碍的“严重程度”
            temp_Num=CBHBUM;
    }

    CBHBUM=temp_Num;  //更新CBHBUM
    float temp=0;    //去图像正中心，车头前面的一小块区域(175个像素)
    int check_w = 17, check_h = 10; // 定义半宽和高度
    for(int i=-check_w;i<=check_w;++i){  //亮度相加，求平均值得到temp
        for(int j=0;j<check_h*2;++j){
            temp+=AT((IMG_W>>1)+i,Base_starty-5-j);
        }
    }

    temp/=(float)((check_w*2+1)*check_h*2);  //计算平均灰度值
    //如果没有停车Stop_car=0，并且（中心很黑&&跳变很多）又或者是跳变超级多
    if(0==Stop_car&&((temp<OSTU_threshold&&CBHBUM>(Check_Stop_TH/2))||CBHBUM>=Check_Stop_TH)){
        if(fps>=FPS_TH){//如果障碍持续满400帧的话，停车
            Stop_car=1;
            fps=0;
        }
        All_Check=0;//暂停正常巡线
        Block_num++;//进入障碍保护
        if(Stop_car)//如果停车标志位为1，保障保护Block_num设置为10
            Block_num=10;
    }
    if(Block_num>0){  //障碍期间，降低搜索高度，防止误判
        starty=clip(Base_starty-10,0,IMG_H-1);  //往上提一点
        Block_num--;  
    }
    else   //正常位置
        starty=clip(Base_starty-10,0,IMG_H-1);
    //最重大判断：是不是彻底丢线了
    //中心很黑&&左边没线&&右边没线&&白现在很底下||中心太黑(<=5)
    if((temp<OSTU_threshold&&Rep_Lnum<5&&Rep_Rnum<5&&Long_WhiteLine_Y>(IMG_H - 60))||temp<=5){
        Loss_fps++;
        if(Loss_fps>LossTH)//如果丢线够久
            Loss_Line=1;//状态设置为彻底丢线
    }
}

//=================================================================
//最长白列，返回的是最长白点的y坐标
uint8 Long_White(void)
{
    // 1. 确定搜索的中轴线 X 和 起始高度 Y
    int search_x = IMG_W >> 1;     // 160列
    int start_y = IMG_H - 20;      // 230行 (避开车头)
    int y = start_y;
    // 2. 寻找第一个白点（确保从赛道上开始搜）
    bool found_white = false;
    for(int i = 0; i < 60; ++i){
        if(AT(search_x, y - i) > OSTU_threshold){
            y -= i;
            found_white = true;
            break;
        }
    }
    if(!found_white) 
        return (uint8)y; // 没找到白点就返回起始位置
    // 3. 向上爬升，直到遇到黑块（赛道尽头或弯道顶部）
    // 使用步长 2 或 1 提高精度
    while(y > 5){
        // 如果上方点变黑，或者发生剧烈灰度跳变(CBH)
        if(AT(search_x, y - 1) < OSTU_threshold || CBH(AT(search_x, y), AT(search_x, y - 4)) > 50){
            break;
        }
        y--;
    }
    // 4. 更新全局变量，供其他函数参考
    Long_WhiteLine_X = search_x;
    Long_WhiteLine_Y = y;
    // 返回值限制在合理高度范围内
    return (uint8)clip(y, 0, IMG_H - 1);
}
//=================================================================
//加速
void Speed_Change(void)
{
    if(!run)//如果确定了车停止就直接返回
        return;
    SpeedChangeH=SpeedChangeH%4;//循环状态机，加速阶段计数器，实现阶梯式加速
    float Speed_Temp=base_speed;//临时初始速度变量，初始=基础速度
    LongWhite_Speed=clip(LongWhite_Speed, 75, 100);   //限制加速出发门槛的范围，只有赛高长度大于这个值的时候才会开始加速
    //如果白列长度够长的话，并且弯道半径足够大，路比较直时，进入提速逻辑
    if(Long_WhiteLine > LongWhite_Speed && fabs(camera_error.load()) <5.0f){
        //cout<<"SpeedChangeH:  "<<(int)SpeedChangeH<<endl;
        switch(SpeedChangeH){
            case 0:      //保持基准速度，不进行额外加速
                Speed_Temp=base_speed;
                break;
            case 1:   //复合判断
                //如果左右都是直线，按照straight_SpeedK大小比例提速
                //否则按照WhiteLine_SpeedK小比例提速
                if(is_straightL&&is_straightR)
                    // 使用长度差进行加速
                    Speed_Temp += straight_SpeedK * (Long_WhiteLine - LongWhite_Speed);
                else
                    Speed_Temp += WhiteLine_SpeedK * (Long_WhiteLine - LongWhite_Speed);
                break;
            case 2:   //纯直线提速
                if(is_straightL&&is_straightR)
                    Speed_Temp += straight_SpeedK * (Long_WhiteLine - LongWhite_Speed);
                break;
            case 3://基础线段提速，只要路够长就按照WhiteLine_SpeedK提速
                Speed_Temp += WhiteLine_SpeedK * (Long_WhiteLine - LongWhite_Speed);
                break;
            default:
                Speed_Temp=base_speed;
                break;
        }
    }
    else    //否则的话，保持基准速度
        Speed_Temp=base_speed;
    //如果遇到了坡道，速度强制降为Base_Speed-40.0f
    if(Ramp_NULL!=Ramp_type)
        Speed_Temp=base_speed;
    //如果遇到了环岛，速度回归Base_Speed
    if(Circle_NULL!=circle_type)
        Speed_Temp=base_speed;
    //最后通过Set_Target_Speed(Speed_Temp)将计算出的目标速度发送给底层的PID控制器进行
    //cout<<"Speed_Temp: "<<Speed_Temp<<endl;
    Set_Target_Speed(Speed_Temp);
    //base_speed=Speed_Temp;  //更新基础速度为当前计算的速度，供下一次调用参考
}

void banmaxian_center_scan_stop()
{
    static int miss=0; // 连续未检测到斑马线的帧数计数器
    static int count=0; // 连续检测到斑马线的帧数计数器
    static int stage=0; // 检测阶段，0表示未检测到斑马线，1表示检测到斑马线
    int total_change_count = 0; // 中心区域总跳变计数
    int valid_lines_count = 0;  // 满足特征的有效行数
   
    int center_col = IMG_W / 2; // 图像的中心列

    // 扫描第 15 行到第 22 行（靠近车头的近景）
    for (int i = 205; i >= 183; --i){
        if (i >= IMG_H || i < 0)
            continue; // 越界保护

        int line_change_count = 0; // 当前行在中心区域的跳变计数
        // 不扫整行，只从中心列向左40列（center_col - 40）扫到向右40列（center_col + 40）
        int start_j = center_col - 60;
        int end_j = center_col + 60;

        // 安全限幅，防止设置的40列超出图像实际左右边界
        if (start_j < 0)
            start_j = 0;
        if (end_j >= IMG_W - 1)
            end_j = IMG_W - 2;

        for (int j = start_j; j <= end_j; j++){
            // 使用大津法阈值判断相邻像素
            bool current_pixel = (Image_Used[i][j] >= OSTU_threshold);
            bool next_pixel    = (Image_Used[i][j+1] >= OSTU_threshold);

            if (current_pixel != next_pixel){
                line_change_count++; // 记录一次黑白跳变
            }
        }
        // 阈值调整：因为横向只扫了80个像素，所以单行能触及的条纹变少了
        // 正常单色赛道在中心区域是0次跳变（全是白色的赛道）。如果有斑马线，至少会有8次以上的跳变
        if (line_change_count >=8){
            valid_lines_count++;
            total_change_count += line_change_count;
        }
    }
    //cout<<"Valid Lines: "<<valid_lines_count<<" Total Changes: "<<total_change_count<<endl;
    // 判断是否检测到斑马线的条件：至少有10行有效跳变，且总跳变数超过120次
    bool detect = (valid_lines_count >= 12 && total_change_count >= 100);
    // 判断决策：
    // 8行里有5行以上抓到了密集条纹，且总跳变数超过20次（说明正前方有密集的黑白格）
    if (detect){
        count++;
        miss=0; // 重置连续未检测到斑马线的帧数计数器
        if(stage==0&&count>=3){
            stage=1; // 进入检测阶段
            count=0; // 重置计数器，等待连续检测到斑马线
            cout<<"First Stage: Zebra Detected -> WAIT"<<endl;
        }
        else if(stage==1&&count>=4){
            stage=0; // 进入确认阶段
            count=0; // 重置计数器，等待连续检测到斑马线
            Stop_car=1; // 触发停车
            run=0; // 停止运行
            cout<<"Second Stage: Confirmed Zebra Detected -> STOP"<<endl;
        }
    }
    else{
        miss++;
        if(miss>=5){
            count=0; // 重置计数器，等待斑马线消失后再恢复
            miss=0; // 重置连续未检测到斑马线的帧数计数器
            //cout << "!!! [Center Zebra Disappeared] -> RESUME !!!"<<endl;
        }
    }
}

//=================================================================
void Image_Init(void)
{
    cam.start_collect();
    if(!cam.is_cam_opened()){
        printf("Error:Camera Open Failed!\n");
        return;
    }
    //ncnn    
    string model_param="tiny_classifier_fp32.ncnn.param";
    string model_bin="tiny_classifier_fp32.ncnn.bin";
    
    vector<string> labels={"supplies","vehicle","weapon"};
    float mean_vals[3]={123.675f,116.28f,103.53f};
    float norm_vals[3]={0.01712475f, 0.017507f, 0.01742919f};
    ncnn_net_.SetModelPath(model_param, model_bin);
    ncnn_net_.SetInputSize(size, size);
    ncnn_net_.SetLabels(labels);
    ncnn_net_.SetNormalize(mean_vals, norm_vals);

    cout << "正在初始化NCNN模型..." << endl;
    if (ncnn_net_.Init()) {
        cout << "模型加载成功！" << endl;
        model_initialized_ = true;
    } else {
        cout << "警告：模型加载失败！将无法正常识别类别。" << endl;
    }

    gray_pic.create(IMG_H,IMG_W,CV_8UC1);
    raw_pic.create(IMG_H,IMG_W,CV_8UC1);
    ipm_pic.create(IMG_H,IMG_W,CV_8UC1);
}

int Img_Copy(void)
{
    if(run&&cam.get_frame_raw_gray(raw_pic,gray_pic)){
        if(!gray_pic.empty()){
            memcpy(Image_Used,gray_pic.data,IMG_H*IMG_W);
            return 1;
        }
    }
    return 0;
}

void Img_deal(void)
{
    get_OSTU();   //大津法二值化，自动计算得到图像的阈值
    //寻找视野当中最远的白点，判定这条路有多长
    get_highest(&Long_WhiteLine_X,&Long_WhiteLine_Y);
    //丢线检查
    banmaxian_center_scan_stop();
    //Check_Loss();
    //最远点计算，决定当前图像是否有效，如果为全黑，后面所有计算全是浪费计算
    Long_WhiteLine=IMG_H-10-Long_White();  //中心白列长度
    //cout<<"Long_WhiteLine: "<<(int)Long_WhiteLine<<endl;
    Find_Line();   //找线
    //先滤波，左右分别滤波
    blur_points(Left_Line,Left_Num,Blue_Left_Line); //左边线滤波
    blur_points(Right_Line,Right_Num,Blue_Right_Line);  //右边线滤波
    //再进行重采样，让点与点之间的物理距离相等，只有点距相等，后面算出来的角度、曲率才是准的
    resample_points(Blue_Left_Line,Left_Num,Rep_Left_Line,&Rep_Lnum); //左边线重采样
    resample_points(Blue_Right_Line,Right_Num,Rep_Right_Line,&Rep_Rnum);//右边线重采样

    //local_angel先算出每个点的坡度，nums_angle剔除微小的抖动，只保留角度变化最大的点
    //find_corners确认这些点是不是十字路口或者环岛的特征
    //只有线条平滑才能准确计算出某一段路是不是急弯
    local_angel_points(Rep_Left_Line,Rep_Lnum,Left_Angel); //左边线计算每个点角度
    local_angel_points(Rep_Right_Line,Rep_Rnum,Right_Angel);//右边线计算每个点角度
    //非极大值抑制
    nms_angle(Left_Angel,Rep_Lnum,Nms_Left_Angel);//左边线角度非极大值抑制
    nms_angle(Right_Angel,Rep_Rnum,Nms_Right_Angel);//右边线角度非极大值抑制
    find_corners();//寻找角点
    Speed_Change();
    //路径建模，生成虚拟导线
    if(L_Out==LeftP.LP&&LeftP.ID<70)
        //平移得到左中线
        track_leftline(Rep_Left_Line,0,LeftP.ID,Left_Mid_Line,&L_Mid_Num);
    else
        //平移得到左中线    
        track_leftline(Rep_Left_Line,0,Rep_Lnum,Left_Mid_Line,&L_Mid_Num);
    if(L_Out==RightP.LP&&RightP.ID<70)
        //平移得到右中线
        track_rightline(Rep_Right_Line,0,RightP.ID,Right_Mid_Line,&R_Mid_Num);
    else
        //平移得到右中线
        track_rightline(Rep_Right_Line,0,Rep_Rnum,Right_Mid_Line,&R_Mid_Num);
    //最终仲裁，所有线、角点都找到了，这个函数决定本帧图像时信左线还是右线，
    //抑或是进环岛进行特殊处理

    Track_Choose();
}

extern int start_global;
extern int start_test; // 用于测试的全局变量，记录 start 的值
// --- UDP配置 ---
#define SERVER_SYD_IP "10.161.69.213"  // 修改为你电脑的IP地址
#define SERVER_SYD_PORT 8080           // 修改为上位机监听的端口
lq_udp_client syd_udp(SERVER_SYD_IP, SERVER_SYD_PORT); // 实例化对象

#define SERVER_LYC_IP "192.168.43.36"  // 修改为你电脑的IP地址
#define SERVER_LYC_PORT 8080           // 修改为上位机监听的端口
lq_udp_client lyc_udp(SERVER_LYC_IP, SERVER_LYC_PORT); // 实例化对象
cv::Mat show_mat;
void view(void)
{
    for (int y = 0; y < IMG_H; y++) {
                uint8_t* row_ptr = ipm_pic.ptr<uint8_t>(y);
                for (int x = 0; x < IMG_W; x++) {
                    int src_x = (int)mapx[y][x];
                    int src_y = (int)mapy[y][x];
                    if (src_x >= 0 && src_x < IMG_W && src_y >= 0 && src_y < IMG_H) {
                        row_ptr[x] = gray_pic.at<uint8_t>(src_y, src_x);
                    }
                    else {
                        row_ptr[x] = 0;
                    }      
                }
            }
            cv::cvtColor(gray_pic,show_mat, cv::COLOR_GRAY2BGR);
            //cv::line(show_mat,cv::Point(IMG_W/2-60,205),cv::Point(IMG_W/2+60,205),cv::Scalar(0,255,255),1);
            //cv::line(show_mat,cv::Point(IMG_W/2-60,183),cv::Point(IMG_W/2+60,183),cv::Scalar(0,255,255),1);
            //cv::line(show_mat,cv::Point(IMG_W/2-60,205),cv::Point(IMG_W/2-60,183),cv::Scalar(0,255,255),1);
            //cv::line(show_mat,cv::Point(IMG_W/2+60,205),cv::Point(IMG_W/2+60,183),cv::Scalar(0,255,255),1);
                
            // 5. 绘制边线和中线
            // 5.1 绘制原有的左边线 (蓝色)
            /*
            for (int i = 0; i < Rep_Lnum; i++) 
                cv::circle(show_mat, cv::Point((int)Rep_Left_Line[i][0], (int)Rep_Left_Line[i][1]), 1, cv::Scalar(255, 0, 0), -1);
            // 5.2 绘制原有的右边线 (绿色)
            for (int i = 0; i < Rep_Rnum; i++) 
                cv::circle(show_mat, cv::Point((int)Rep_Right_Line[i][0], (int)Rep_Right_Line[i][1]), 1, cv::Scalar(0, 255, 0), -1);
            // 5.1 绘制远端的左边线 (红色)
            for (int i = 0; i < Far_Left_Num; i++) 
                cv::circle(show_mat, cv::Point((int)FarLeft_Line[i][0], (int)FarLeft_Line[i][1]), 1, cv::Scalar(0, 0, 255), -1);
            // 5.2 绘制远端的右边线 (黄色)
            for (int i = 0; i < Far_Right_Num; i++) 
                cv::circle(show_mat, cv::Point((int)FarRight_Line[i][0], (int)FarRight_Line[i][1]), 1, cv::Scalar(0, 255, 255), -1);
            // 5.3 绘制中线 (紫色)
            for(int i=0;i<(L_Mid_Num);i++){
              cv::circle(show_mat,cv::Point((int)Left_Mid_Line[i][0],(int)Left_Mid_Line[i][1]),1,cv::Scalar(0, 0, 255),-1);
            }
            //右中线(黄色)
            for(int i=0;i<(R_Mid_Num);i++){
              cv::circle(show_mat,cv::Point((int)Right_Mid_Line[i][0],(int)Right_Mid_Line[i][1]),1,cv::Scalar(0, 255, 255),-1);
            }*/
            //中线(紫色)
            for(int i=0;i<(*MidnumPts);i++){
              cv::circle(show_mat,cv::Point(mapx[(int)mid_pts[i][1]][(int)mid_pts[i][0]],mapy[(int)mid_pts[i][1]][(int)mid_pts[i][0]]),1,cv::Scalar(255, 0, 255),-1);
            }
            //target_detect_deal();
            // --- 6. 发送显示 ---
            lyc_udp.udp_send_image(show_mat, 80);
}

//=================================================================

//目标识别部分
//记录黑白跳变发生的位置
int target_trigger_y=-1;
int target_trigger_x=-1;

cv::Mat hsv_img;   //HSV图像
cv::Mat red_mask;  //红色mask
cv::Mat target_crop;//裁剪结果

bool target_detect_flag=false;   //当前是否进入目标检测流程
//检测当前状态
enum TargetState{
    NO_TARGET,
    CHECK_RED,
    CLASSFY
};
TargetState target_state=NO_TARGET;
//NCNN结果举例
enum Image_Type{
    unknown,
    supplies,
    vehicle,
    weapon
};
Image_Type current_target=unknown;

//切线斜率

float angle;
int trigger=-1;
int mid_line_detect(void)
{  
    static int x_last=0,y_last=0;
    static int x_last_1=0,y_last_1=0;
    switch(LRType){
        case MID_Left:  
            Center_Line = Left_Mid_Line;  
            Center_Line_Num = &L_Mid_Num;
            break;
        case MID_Right:
            Center_Line = Right_Mid_Line;
            Center_Line_Num = &R_Mid_Num;
            break;
        case MID_Ramp:  
            Center_Line = RampMID;
            Center_Line_Num = &RampMIDNum;
            break;
        default:
            break;
    }
    if(*Center_Line_Num<5){
        return -1;
    }
    for(int i=5;i<*Center_Line_Num;i++){
        int x=mapx[(int)Center_Line[i][1]][(int)Center_Line[i][0]];
        int y=mapy[(int)Center_Line[i][1]][(int)Center_Line[i][0]];

        if(AT(x,y)>OSTU_threshold){
            x_last_1=x_last;
            y_last_1=y_last;
            x_last=x;
            y_last=y;
            continue;
        }

        if(AT(x,y)<OSTU_threshold&&AT(x_last,y_last)>OSTU_threshold&&AT(x_last_1,y_last_1)>OSTU_threshold){
            target_trigger_x=x;
            target_trigger_y=y;
            trigger=i;
            //cout<<trigger<<endl;
            //cout<<"target_trigger_y:  "<<target_trigger_y<<"Center_Line_Num"<<*Center_Line_Num<<endl;
            return 1;
        }

    }
    return 0;
}

//初始化红色HSV阈值
const cv::Scalar lower_red1_ = cv::Scalar(0, 70, 45);
const cv::Scalar upper_red1_ = cv::Scalar(10, 255, 255);
const cv::Scalar lower_red2_ = cv::Scalar(170, 70, 45);
const cv::Scalar upper_red2_ = cv::Scalar(180, 255, 255);

// 初始化白色赛道 HSV 阈值
const cv::Scalar lower_white_ = cv::Scalar(0, 0, 160);
const cv::Scalar upper_white_ = cv::Scalar(180, 35, 255);
cv::Mat mask1_, mask2_, red_mask_, last_cropped_img_;

//专门提取红块掩码
void get_red_mask(cv::Mat&hsv_input,cv::Mat& output_mask)
{
    //提取红色的第一个色段
    cv::inRange(hsv_input,lower_red1_,upper_red1_,mask1_);
    //提取红色的第二个色段
    cv::inRange(hsv_input,lower_red2_,upper_red2_,mask2_);
    //将两个掩码进行逻辑或运算，合并成一个完整的红色区域图
    cv::bitwise_or(mask1_,mask2_,output_mask);
}

cv::RotatedRect current_rRect;
cv::RotatedRect final_refind_rRect;
float zoom=0.25f;
cv::Rect QuickDetect(int center_x,int center_y,const cv::Mat&hsv_img)
{
    //算切向角
    int y_idx=trigger;
    //取目标点上下附近的点来求导，计算赛道切线方向
    int y_upper=std::max(10,std::min(y_idx+20,*Center_Line_Num-1));
    int y_lower=std::max(10,std::min(y_idx,*Center_Line_Num-1));
    //dx，dy是切线的偏移量，atan2计算弧度，乘上RAD_TO_DEG转为角度
    float dx=Center_Line[y_upper][0]-Center_Line[y_lower][0];
    float dy=Center_Line[y_upper][1]-Center_Line[y_lower][1];
    //cout<<dx<<"    "<<dy<<endl;
    float rad=atan2(dy,dx);
    angle=rad*RAD_TO_DEG;
    
    cv::Size2f rSize(55.0f-zoom*y_idx, 45.0f-zoom*y_idx); 
    
    // 2. 创建 RotatedRect，注意：此时 center 是中心点
    // 如果 trigger 是底边中点，那么中心点需要向上平移 height/2
    float height = rSize.height;
    float center_offset_x = (height / 2.0f) * cos(rad);
    float center_offset_y = -(height / 2.0f) * sin(rad);
    
    cv::Point2f center_point(center_x + center_offset_x, center_y - center_offset_y);
    current_rRect = cv::RotatedRect(center_point, rSize, angle);

    // 3. 利用 boundingRect 获得绝对安全的轴对齐矩形 (不会掉信息！)
    cv::Rect safe_roi = current_rRect.boundingRect();
    
    // 4. 修正边界，防止越界
    safe_roi &= cv::Rect(0, 0, hsv_img.cols, hsv_img.rows);

    // 5. 裁剪并验证
    cv::Mat hocal_bgr=hsv_img(safe_roi);
    cv::Mat local_hsv;
    cv::cvtColor(hocal_bgr,local_hsv,cv::COLOR_BGR2HSV);
    cv::Mat local_mask;
    get_red_mask(local_hsv, local_mask);
    
    int red_pixels = cv::countNonZero(local_mask);
    int total_pixels = local_mask.rows * local_mask.cols;
    
    if(total_pixels > 0 && (float)red_pixels > total_pixels * 0.05 && (float)red_pixels < total_pixels * 0.85){
        return safe_roi;
    }
    return cv::Rect(0,0,0,0);
}

cv::Mat CropAndPerspective(const cv::Mat& src, cv::RotatedRect rRect)
{
    // 1. 获取 RotatedRect 的四个顶点（这是最真实的物理边框）
    cv::Point2f src_pts[4];
    rRect.points(src_pts);

    // 2. 定义目标矩形的四个顶点（即“拉正”后的样子）
    // 宽度和高度取 rRect 的长宽
    float w = rRect.size.width;
    float h = rRect.size.height;
    cv::Point2f dst_pts[4] = {   
        cv::Point2f(h, w),     // 右下
        cv::Point2f(0, w),    // 左下        
        cv::Point2f(0, 0),    // 左上
        cv::Point2f(h, 0)    // 右上
    };

    // 3. 计算透视变换矩阵
    cv::Mat trans_mat = cv::getPerspectiveTransform(src_pts, dst_pts);

    // 4. 执行透视变换
    cv::Mat rotated_crop;
    cv::warpPerspective(src, rotated_crop, trans_mat, cv::Size((int)w, (int)h), cv::INTER_LINEAR);

    return rotated_crop;
}

//图片旋转
cv::Mat CropAndRotate(const cv::Mat&src,cv::Rect roi,float angle)
{
    //从原图中切出ROI区域
    cv::Mat crop=src(roi);
    //以切出的矩形中心为旋转基准点
    cv::Point2f center(crop.cols/2.0f,crop.rows/2.0f);
    //根据计算出来的angle获取仿射变换矩阵
    //1.0f是缩放比例，即保持原大小不变
    cv::Mat rot_mat=cv::getRotationMatrix2D(center,(90.0+angle),1.0f);
    //执行旋转变换
    //INTER_LINEAR(双线性插值)是性能与效果的最佳平衡点
    cv::Mat rotated_crop;
    cv::warpAffine(crop,rotated_crop,rot_mat,crop.size(),cv::INTER_LINEAR,cv::BORDER_REPLICATE);
    //返回摆正后的局部图像
    return rotated_crop;
}

//进行裁剪
cv::Mat ExtractSymbolOnly_ByJump(const cv::Mat& rotated_crop)
{
    cv::Mat hsv_roi, symbol_mask;
    cv::cvtColor(rotated_crop, hsv_roi, cv::COLOR_BGR2HSV);
    get_red_mask(hsv_roi, symbol_mask);

    // 1. 先进行一次形态学开运算 (Opening)，去掉中间连接的细小噪声
    // 这样红块之间如果没有物理接触，就会彻底断开
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(symbol_mask, symbol_mask, cv::MORPH_OPEN, kernel);

    vector<int> row_counts(symbol_mask.rows);
    for(int i = 0; i < symbol_mask.rows; ++i){
        row_counts[i] = cv::countNonZero(symbol_mask.row(i));
    }

    // 2. 寻找“分界线”
    // 我们从中间向上找，找到第一个 row_counts 为 0 或极小的值
    // 这代表基座和标志之间的“空隙”
    int bottom_edge = symbol_mask.rows;
    for(int i = symbol_mask.rows - 1; i > symbol_mask.rows / 2; --i) {
        // 如果该行红点数很少，说明这里是两个块之间的空隙
        if(row_counts[i] < (symbol_mask.cols * 0.05)){ // 阈值设为列宽的5%
            bottom_edge = i;
            break;
        }
    }

    // 如果没找到明显的空隙，则保留原图 (为了安全)
    if (bottom_edge == symbol_mask.rows) bottom_edge = symbol_mask.rows;

    cv::Rect symbol_rect(0, 0, symbol_mask.cols, bottom_edge);
    symbol_rect &= cv::Rect(0, 0, rotated_crop.cols, rotated_crop.rows);
    
    return rotated_crop(symbol_rect);
}

void getTargetType(const cv::Mat&cropped_img)
{
    if(cropped_img.empty()||!model_initialized_){
        return;
    }

    string result=ncnn_net_.Infer(cropped_img);

    if(result=="supplies")
        current_target=supplies;
    else if(result=="vehicle")
        current_target=vehicle;
    else if(result=="weapon")
        current_target=weapon;
    else
        current_target=unknown;
    
    //cout<<"推理结果："<<result<<endl;
        
}

/**
 * @param img           要绘制的底图 (建议传 raw_pic)
 * @param roi           QuickDetect 得到的原始矩形
 * @param angle         切线角
 * @param bottom_edge   ExtractSymbolOnly_ByJump 得到的截断位置 (相对坐标)
 * @param rotated_img   ExtractSymbolOnly_ByJump 之后的那张摆正后的图 (用于画局部截断线)
 */
void VisualizeDetection(cv::Mat& img, cv::Rect roi, float angle, int bottom_edge, cv::Mat& rotated_img)
{
    // 直接获取 RotatedRect 的四个顶点
    cv::Point2f vertices[4];
    current_rRect.points(vertices);
    
    // 绘制绿色的斜框
    for (int i = 0; i < 4; i++) {
        cv::line(img, vertices[i], vertices[(i + 1) % 4], cv::Scalar(0, 255, 0), 2);
        cv::circle(img, vertices[i], 5, cv::Scalar(0, 0, 255), -1);
    }

    // --- 第二部分：在摆正后的局部图中绘制红色截断线 ---
    // 这行线清晰展示了 ExtractSymbolOnly_ByJump 的工作效果
    cv::circle(show_mat,cv::Point(target_trigger_x,target_trigger_y),4,cv::Scalar(255,0,255),3);
}

void target_detect_deal(void)
{
    if(mid_line_detect()==1){
        cv::Rect found_roi = QuickDetect(target_trigger_x, target_trigger_y, raw_pic);
        //cout<<angle<<endl;
        if(found_roi.width > 0){
            cv::Mat rotated = CropAndPerspective(raw_pic, current_rRect);
            cv::Mat final_img = ExtractSymbolOnly_ByJump(rotated);
            cv::resize(final_img,last_cropped_img_,cv::Size(96,96));
            getTargetType(last_cropped_img_);
            // 一目了然：绿框是搜索范围，红线是截断位置，红点是角点
            //lyc_udp.udp_send_image(final_img, 80);
            //VisualizeDetection(show_mat, found_roi, angle,final_img.rows,rotated);
            
            // ... 喂给模型 ...
        }
    }
}