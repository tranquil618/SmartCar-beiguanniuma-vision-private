#include "Cross.h"
#include "Define.h"
//#include"Screen.h"
#include "Drive.hpp"
#include "Circle.h"
#include "Rot.h"
#include<iostream>
using namespace std;
float FarLeft_Line[MAX_POINTS][2];
int Far_Left_Num;
float FarRight_Line[MAX_POINTS][2];
int Far_Right_Num;

float FarBlueLeft_Line[MAX_POINTS][2];
float FarBlueRight_Line[MAX_POINTS][2];

float FarRepLeft_Line[MAX_POINTS][2];
int FarRepLnum;
float FarRepRight_Line[MAX_POINTS][2];
int FarRepRnum;

float FarLeftAngel[MAX_POINTS];
float FarRightAngel[MAX_POINTS];

float FarNmsLeftAngel[MAX_POINTS];
float FarNmsRightAngel[MAX_POINTS];

LP_t FarLeftP={L_NULL,0},FarRightP={L_NULL,0};
Cross_t CrossType=Cross_NULL;
uint16 CrossEnc_TH_low=1500;
uint16 CrossEnc_TH_high=2060;
float Cross_Dis=45.0f;
uint8 Out_CrossTH_Min=20;
uint8 Out_CrossTH_Max=45;
int start_test=0;
// --- 新增：十字路口近端航向锁定与低通滤波变量 ---
static float Locked_K = 0.0f;
static float Locked_B = 0.0f;
static float filtered_K = 0.0f;
static float filtered_B = 0.0f;
static const float Cross_Filter_Alpha = 0.6f; // 滤波系数：0.5-0.7 之间，越小越平滑


uint8 Check_LeftCross(void)
{
    //计算当前左边线的切线方向
    //只有知道了车子现在是怎么歪着进十字的，才能够准确算出对面赛道的位置
    int half=(LeftP.ID+Rep_Lnum)/2;//LeftP.ID：当前左边线识别到的角点（Corner）索引；取当前边缘线的中点
    int im=LeftP.ID,ip=Rep_Lnum-1;
    float dx=Rep_Left_Line[ip][0]-Rep_Left_Line[im][0];
    float dy=Rep_Left_Line[ip][1]-Rep_Left_Line[im][1];
    //归一化方向向量
    float dn=fast_invsqrt(dx*dx+dy*dy);
    //单位方向向量，代表了当前赛道的朝向
    dx*=dn;
    dy*=dn;
    //物理坐标下的盲跳，利用垂直向量的概念，在物理坐标系中，如果沿着赛道方向向量
    //旋转90度并且平移Cross_Dis，能够从当前的左拐角点跳到对岸的左边缘
    float x=0,y=0;
    x=Rep_Left_Line[half][0]-dy*Cross_Dis;
    y=Rep_Left_Line[half][1]+dx*Cross_Dis;
    //坐标变换，逆透视还原，坐标会投影
    uint16 invX=(uint16)inv_rot_x(x,y);
    uint16 invY=(uint16)inv_rot_y(x,y);
    //边缘对其与二次寻找
    //在跳到的位置纵向方向附近找一下，哪里是黑白交界，找到真正的边缘点之后，修正invY
    for(int i=0;i<MIN(40,invY-1);++i){
        if(AT(invX,invY-i)>OSTU_threshold&&AT(invX,invY-i-1)<OSTU_threshold){
            invY-=i;
            break;
        }
    }
    //二次扫线爬虫
    //从跳过去的那个点开始，重新沿着边缘爬。
    findline_lefthand_adaptive(invX,invY,FarLeft_Line,&Far_Left_Num);
    blur_points(FarLeft_Line,Far_Left_Num,FarBlueLeft_Line);
    resample_points(FarLeft_Line,Far_Left_Num,FarRepLeft_Line,&FarRepLnum);
    //远端特征分析
    local_angel_points(FarRepLeft_Line,FarRepLnum,FarLeftAngel);
    nms_angle(FarLeftAngel,FarRepLnum,FarNmsLeftAngel);
    FarLeftP.LP=L_NULL;
    FarLeftP.ID=(uint8)FarRepLnum;
    uint8 fornum=MIN(FarRepLnum,55);
    for(uint8 i=0;i<fornum;++i){
        if(FarNmsLeftAngel[i]==0)
            continue;
        int im1=clip(i-ANGLE_DIS,0,FarRepLnum-1);
        int ip1=clip(i+ANGLE_DIS,0,FarRepLnum-1);
        //角度置信度
        float conf=fabs(FarLeftAngel[i])-(fabs(FarLeftAngel[im1])+fabs(FarLeftAngel[ip1]))/2;
        if(conf>1.22f&&conf<1.92f){//69.9-110
            //如果远端线上发现了一个接近90度的拐角，那么十字路口实锤
            if(FarLeftAngel[i]>0){
                FarLeftP.LP=L_In;
                FarLeftP.ID=i;
            }
            else{
                FarLeftP.LP=L_Out;
                FarLeftP.ID=i;
            }
        }
        if(L_NULL!=FarLeftP.LP)
            break;
    }
    return (uint8)(L_NULL!=FarLeftP.LP);
}

uint8 Check_RightCross(void)
{
    int half=(LeftP.ID+Rep_Rnum)/2;
    int im=LeftP.ID,ip=Rep_Rnum-1;
    float dx=Rep_Right_Line[ip][0]-Rep_Right_Line[im][0];
    float dy=Rep_Right_Line[ip][1]-Rep_Right_Line[ip][1];
    float dn=fast_invsqrt(dx*dx+dy*dy);
    dx*=dn;
    dy*=dn;
    float x=0,y=0;
    x=Rep_Right_Line[half][0]+dy*Cross_Dis;
    y=Rep_Right_Line[half][1]-dx*Cross_Dis;
    uint16 invX=(uint16)inv_rot_x(x,y);
    uint16 invY=(uint16)inv_rot_y(x,y);
    for(int i=0;i<MIN(40,invY-1);++i){
        if(AT(invX,invY-i)>OSTU_threshold&&AT(invX,invY-i-1)<OSTU_threshold){
            invY-=i;
            break;
        }
    }
    findline_righthand_adaptive(invX,invY,FarRight_Line,&Far_Right_Num);
    blur_points(FarRight_Line,Far_Right_Num,FarBlueRight_Line);
    resample_points(FarRight_Line,Far_Right_Num,FarRepRight_Line,&FarRepRnum);
    local_angel_points(FarRepRight_Line,FarRepRnum,FarRightAngel);
    nms_angle(FarRightAngel,FarRepRnum,FarNmsRightAngel);
    FarRightP.LP=L_NULL;
    uint8 fornum=MIN(FarRepRnum,55);
    for(uint8 i=0;i<fornum;++i){
        if(FarNmsRightAngel[i]==0)
            continue;
        int im1=clip(i-ANGLE_DIS,0,FarRepRnum-1);
        int ip1=clip(i+ANGLE_DIS,0,FarRepRnum-1);
        float conf=fabs(FarRightAngel[i])-(fabs(FarRightAngel[im1])+fabs(FarRightAngel[ip1]))/2;
        if(conf>1.22f&&conf<1.92f){//69.9-110
            if(FarRightAngel[i]>0){
                FarRightP.LP=L_In;
                FarRightP.ID=i;
            }
            else{
                FarRightP.LP=L_Out;
                FarRightP.ID=i;
            }
        }
        if(L_NULL!=FarRightP.LP)
            break;
    }
    return (uint8)(L_NULL!=FarRightP.LP);
}

//十字路口初步识别与分类
void Cross_Check(void)
{
    //首先标志位定义，并且为static
    static uint8 Checkflag=1;
    //如果当前车辆已经处于环岛状态，强行关闭十字路口检测
    if(Circle_NULL!=circle_type)
        Checkflag=0;
    else    
        Checkflag=1;
    //左右边缘有效点的个数
    FarRepLnum=FarRepRnum=0;
    
    if(All_Check&&Cross_NULL==CrossType&&Checkflag){
        //如果现在不是圆环状态，并且左右两边同时检测到了外角点，并且角点的位置位于中轴线以下
        //这是标准的正入十字路口特征
        if(L_Out==LeftP.LP&L_Out==RightP.LP&&(LeftP.ID<25||RightP.ID<25)){
            CrossType=Cross_ALL;
            //Buzzer_OpenClose(1);
        }
        //右侧丢线，左侧有角点
        else if((L_NULL==RightP.LP||Rep_Rnum<40)&&L_Out==LeftP.LP&&0==is_straightR&&LeftP.ID<25){
            //调用远端二次扫描
            if(Check_LeftCross()){
                CrossType=Cross_Left;
                //Buzzer_OpenClose(1);
            }
        }
        //如果左侧丢线，右侧有角点
        else if((L_NULL==LeftP.LP||Rep_Lnum<40)&&L_Out==RightP.LP&&0==is_straightL&&RightP.ID<25){
            //同样调用远端二次扫描
            if(Check_RightCross()){
                CrossType=Cross_Right;
                //Buzzer_OpenClose(1);
            }
        }
        //否则的话就判定为不是十字
        else   
            CrossType=Cross_NULL;
    }
    //如果当前赛道识别出来不对劲，直接判定为没有十字
    else if(0==All_Check)
        CrossType==Cross_NULL;
}


void Cross_Path_Interpolation(float (*target_mid_line)[2], int *target_num) {
    if (*target_num < 5) return; // 远端点数太少不补线

    // 1. 获取近端起始点（车头中心）
    float start_x = mapx[235][160]; 
    float start_y = mapy[235][160] + 14.0f; 

    // 2. 获取远端重建中线的第一个有效点
    float end_x = target_mid_line[0][0];
    float end_y = target_mid_line[0][1];

    // 3. 计算插值点数（根据物理距离决定，假设每 2cm 补一个点）
    float dist = sqrt(pow(end_x - start_x, 2) + pow(end_y - start_y, 2));
    int insert_count = (int)(dist / 2.0f); 
    insert_count = clip(insert_count, 2, 20); // 限制补点数量

    // 4. 将原有远端点后移，腾出空间给插值点
    for (int i = *target_num - 1; i >= 0; i--) {
        if (i + insert_count < MAX_POINTS) {
            target_mid_line[i + insert_count][0] = target_mid_line[i][0];
            target_mid_line[i + insert_count][1] = target_mid_line[i][1];
        }
    }

    // 5. 线性插值填充近端空白
    for (int i = 0; i < insert_count; i++) {
        float ratio = (float)i / insert_count;
        target_mid_line[i][0] = start_x + (end_x - start_x) * ratio;
        target_mid_line[i][1] = start_y + (end_y - start_y) * ratio;
    }

    // 6. 更新总点数
    *target_num = clip(*target_num + insert_count, 0, MAX_POINTS - 1);
}

static inline int Check_Out(void)
{
    int num=0;//计数器，统计底端一行白色像素点的数量
    for(int i=1;i<IMG_W-1;++i){
        if(AT(i,IMG_H-10)>=OSTU_threshold)
            num++;
    }
    return num;
}



//十字路口处理的核心执行状态机
void Cross_Loop(void)
{
    //静态局部变量
    static int64 tempS=0;//临时路程，记录进入十字瞬间的里程基准
    static uint8 temp=0;//临时标志位，触发锁，确保初始化动作只执行一次
    static int Check=0;//当前帧底部像素值统计，储存当前这一帧Check_Out扫描到的白点数量
    static int Last_Check=0;//上一帧底部像素统计值
    Cross_Check();
    if(Cross_NULL!=CrossType){
        circle_type=Circle_NULL;//进入十字路口状态，强行关闭圆环状态
    }
    //状态触发阶段
    switch(CrossType){
        case Cross_Left:
            //如果角点足够近，将状态机切换为正式进入路口内部
            if(LeftP.ID<5){
                CrossType=Cross_IN;
                tempS=Get_IENC();//记录当前编码器数值，表示从现在开始要盲开一段距离
                temp=1;
            }
            break;
        case Cross_Right:
            //如果角点足够近，将状态机切换为正式进入路口内部
            if(RightP.ID<5){
                CrossType=Cross_IN;
                tempS=Get_IENC();
                temp=1;
            }
            break;
        case Cross_ALL:
            //两端都找到的话，也是用样的方式进行判断，左右都进行判断
            if(RightP.ID<5&&L_NULL!=RightP.LP){
                CrossType=Cross_IN;
                tempS=Get_IENC();
                temp=1;
            }
            if(LeftP.ID<5&&L_NULL!=LeftP.LP){
                CrossType=Cross_IN;
                tempS=Get_IENC();
                temp=1;
            }
            break;
        case Cross_IN: {
            if(temp&&L_Mid_Num<5&&R_Mid_Num<5){
                L_Mid_Num=0;
                R_Mid_Num=0;
            }
            Cross_Refind_Line();
            Last_Check = Check;
            Check = Check_Out();

            bool dist_ok = (Get_IENC() - tempS > CrossEnc_TH_low);
            //cout<<"(Get_IENC()-tempS):  "<<(Get_IENC()-tempS)<<endl;
            bool pixel_jump = (abs(Check - Last_Check) > Out_CrossTH_Min);
            bool force_out = (Get_IENC() - tempS > CrossEnc_TH_high);

            if ((dist_ok && pixel_jump) || force_out) {
                CrossType = Cross_NULL;
                // 彻底退出时才重置 temp，为下次进十字做准备
                temp = 0; 
            }
            break;
        }
        default:
            temp=0;
            break;
        }
}

//远端角点检测器
void find_Farcorners(void)
{
    //清除旧记忆，在开始检测之前先将远端左右角点的属性设置为无效
    //保证每一帧都是新鲜的
    FarLeftP.LP=L_NULL,FarRightP.LP=L_NULL;
    FarLeftP.ID=(uint8)FarRepLnum,FarRightP.ID=(uint8)FarRepRnum;
    //fornum最多只检查前55个点，因为十字路口角点通常出现在线条的前半段
    uint8 fornum=MIN(FarRepLnum,55),i;
    for(i=0;i<fornum;++i){
        //只处理经过非极大值抑制的角度特征值
        if(FarNmsLeftAngel[i]==0)
            continue;
        //取当前点i的角度，减去它前后ANGLE_DIS距离处两个点角度的平均值
        int im1=clip(i-ANGLE_DIS,0,FarRepLnum-1);
        int ip1=clip(i+ANGLE_DIS,0,FarRepLnum-1);
        //计算得到conf置信度值
        float conf=fabs(FarNmsLeftAngel[i])-(fabs(FarNmsLeftAngel[im1])+fabs(FarNmsLeftAngel[ip1]))/2;
        //阈值判定，如果conf落在这个区间，说明抓到了一个接近90度的拐角，
        //根据角度的正负值确定它是内角还是外角
        if(conf>1.05f&&conf<2.01f){//65-115
            if(FarLeftAngel[i]>0){//正角度为内角
                FarLeftP.LP=L_In;
                FarLeftP.ID=i;
            }
            else{//外角度为外角
                FarLeftP.LP=L_Out;
                FarLeftP.ID=i;
            }
        }//如果确定了有角，退出循环
        if((L_NULL!=FarLeftP.LP))
            break;
    }
    //对右边也来进行一次和左边同样的操作
    fornum=MIN(FarRepRnum,55);
    for(i=0;i<fornum;++i){
        if(FarNmsRightAngel[i]==0)
            continue;
        int im1=clip(i-ANGLE_DIS,0,FarRepRnum-1);
        int ip1=clip(i+ANGLE_DIS,0,FarRepRnum-1);
        float conf=fabs(FarNmsRightAngel[i])-(fabs(FarNmsRightAngel[im1])+fabs(FarNmsRightAngel[ip1]))/2;
        if(conf>1.05f&&conf<2.01f){//65-115
            if(FarRightAngel[i]<0){
                FarRightP.LP=L_In;
                FarRightP.ID=i;
            }
            else{
                FarRightP.LP=L_Out;
                FarRightP.ID=i;
            }
        }
        if((L_NULL!=FarRightP.LP))
            break;
    }
}

int start_global;
//远端引导线重建
uint8 Cross_Refind_Line(void)
{
    //寻找对面赛道的黑白边界
    int x1=160,x2=160;
    //动态确定扫描起始高度
    int start=IMG_H/4*3;
    int dynamic_offset=75;
    //如果左右都有特殊状态
    if(L_NULL!=RightP.LP&&L_NULL!=LeftP.LP){
        //如果右边找到的位置比左边高，取角点坐标
        if(RightP.ID>LeftP.ID)
            start=mapy[(int)Rep_Right_Line[RightP.ID][1]-dynamic_offset][(int)Rep_Right_Line[RightP.ID][0]];
        //否则的话使用左边的位置进行变换
        else
            start=mapy[(int)Rep_Left_Line[LeftP.ID][1]-dynamic_offset][(int)Rep_Left_Line[LeftP.ID][0]];
    //    cout<<"type:  "<<1<<endl;
    }
    //如果右边有特殊状态，左边丢线
    else if(L_NULL!=RightP.LP){
        start=mapy[(int)Rep_Right_Line[RightP.ID][1]-dynamic_offset][(int)Rep_Right_Line[RightP.ID][0]];
    //    cout<<"type:  "<<2<<endl;
    }
    //如果左边有特殊状态，右边丢线
    else if(L_NULL!=LeftP.LP){
        start=mapy[(int)Rep_Left_Line[LeftP.ID][1]-dynamic_offset][(int)Rep_Left_Line[LeftP.ID][0]];
    //    cout<<"type:  "<<3<<endl;
    }
    //其他的话就直接取(94,65)的坐标作为start起始坐标
    else{    
        start=60;
    //    cout<<"type:  "<<4<<endl;
    }
    
    start_test=start;//测试用，记录start的值，看看动态调整起始高度的功能是否正常
    //cout<<Long_WhiteLine_X<<endl;
    //寻找对面赛道的黑白边界
    //先向左找左边缘
    for(int i=Long_WhiteLine_X;i>Block_Size/2+1;--i){
        if(AT(i,start)>=OSTU_threshold&&AT(i-1,start)<OSTU_threshold){
            x1=i;
            break;
        }
    }
    //再向右找右边缘
    for(int i=Long_WhiteLine_X;i<IMG_W-Block_Size/2-1;++i){
        if(AT(i,start)>=OSTU_threshold&&AT(i+1,start)<OSTU_threshold){
            x2=i;
            break;
        }
    }
    /*
    if(x1<50||x2>270){
        x1=160;
        x2=160;
    }*/
    start_global=start;
    //cout<<"x1:  "<<x1<<"  x2:  "<<x2<<endl;
    //远端二次爬线与处理
    //八邻域+三角滤波+重采样插值
    Far_Left_Num=Far_Right_Num=0;
    findline_lefthand_adaptive(x1,start,FarLeft_Line,&Far_Left_Num);//存在黑白跳变，寻找左线
    findline_righthand_adaptive(x2,start,FarRight_Line,&Far_Right_Num);//存在黑白跳变，寻找右线
    blur_points(FarLeft_Line,Far_Left_Num,FarBlueLeft_Line);//左边线滤波
    blur_points(FarRight_Line,Far_Right_Num,FarBlueRight_Line);//右边线滤波
    resample_points(FarBlueLeft_Line,Far_Left_Num,FarRepLeft_Line,&FarRepLnum);//左边线重采样
    resample_points(FarBlueRight_Line,Far_Right_Num,FarRepRight_Line,&FarRepRnum);//右边线重采样
    //路径重建，将找到的远端边缘线向内侧平移半个赛道宽度，生成远端中线
    track_leftline(FarRepLeft_Line,0,FarRepLnum,Left_Mid_Line,&L_Mid_Num);//平移得到左中线
    track_rightline(FarRepRight_Line,0,FarRepRnum,Right_Mid_Line,&R_Mid_Num);//平移得到右中线
    //cout<<"L_Mid_Num: "<<L_Mid_Num<<"  R_Mid_Num:  "<<R_Mid_Num<<endl;
    // 执行补线
    if (L_Mid_Num > R_Mid_Num) {
        Cross_Path_Interpolation(Left_Mid_Line, &L_Mid_Num);
        LRType=MID_Left;
    } 
    else {
        Cross_Path_Interpolation(Right_Mid_Line, &R_Mid_Num);
        LRType=MID_Right;
    }
    //cout<<"LRType:  "<<LRType<<endl;
}