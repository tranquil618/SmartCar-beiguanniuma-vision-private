/*
* Recognition.cpp
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

//外部变量
extern LQ_NCNN ncnn_net_;
extern cv::Mat show_mat;
extern cv::Mat raw_pic;
extern lq_udp_client lyc_udp;
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
enum BypassState{
    STATE_IDLE=0,         //未触发绕行，正常检测/投票
    STATE_DECEL_VOTE,     //减速+投票
    STATE_BYPASS_BLIND,   //切出与盲行阶段(惯导+最长白列偏航角)
    STATE_VISION_RECOVERY //视觉找回线
};

//当前状态变量(初始为IDLE)
BypassState current_bypass_state=STATE_IDLE;
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
    int y_upper=std::max(10,std::min(y_idx+10,*Center_Line_Num-1));
    int y_lower=std::max(10,std::min(y_idx,*Center_Line_Num-1));
    //dx，dy是切线的偏移量，atan2计算弧度，乘上RAD_TO_DEG转为角度
    float dx=Center_Line[y_upper][0]-Center_Line[y_lower][0];
    float dy=Center_Line[y_upper][1]-Center_Line[y_lower][1];
    //cout<<dx<<"    "<<dy<<endl;
    float rad=atan2(dy,dx);
    angle=rad*RAD_TO_DEG;
    
    cv::Size2f rSize(60.0f-zoom*y_idx, 45.0f-zoom*y_idx); 
    
    // 创建 RotatedRect，注意：此时 center 是中心点
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

cv::Mat CropAndPerspective(const cv::Mat& src, cv::RotatedRect rRect, int y_idx)
{
    // 1. 获取 RotatedRect 的四个顶点
    cv::Point2f src_pts[4];
    rRect.points(src_pts);

    // 2. 动态计算目标画布大小 (依据距离远近自适应分辨率)
    // 基础尺寸：你可以设定一个标准的期望输出大小，例如期望统一归一化到 128x128 或者是根据物理长宽动态缩放
    float base_w = rRect.size.width;
    float base_h = rRect.size.height;

    // 引入距离因子：y_idx 越大代表距离越近（假设你的坐标系里y往下是增大）
    // 或者用 rRect 的实际高度在图像中所占的像素来决定输出画布大小
    float scale_factor = 1.0f;
    
    // 示例：如果目标在远处（rRect 像素较小），我们适当放大输出画布上限；
    // 如果目标在近处，限制最大输出尺寸，避免过采样消耗算力。
    float current_diagonal = std::hypot(base_w, base_h);
    
    // 设定一个固定的理想网络输入像素尺度（比如 NCNN 模型如果最终要缩放到 96x96，
    // 我们可以让透视变换直接输出一个适中的清晰尺寸，如 128x128，再给模型缩放）
    int target_size = 256; 

    cv::Point2f dst_pts[4] = {              
        cv::Point2f(target_size, target_size),   // 右下
        cv::Point2f(0, target_size),             // 左下        
        cv::Point2f(0, 0),                       // 左上
        cv::Point2f(target_size, 0)              // 右上
    };

    // 3. 计算透视变换矩阵
    cv::Mat trans_mat=cv::getPerspectiveTransform(src_pts, dst_pts);

    // 4. 执行透视变换（输出统一大小的清晰图，方便后续神经网络提取特征）
    cv::Mat rotated_crop;
    cv::warpPerspective(src,rotated_crop,trans_mat,cv::Size(target_size,target_size),cv::INTER_AREA); 
    // 注意：缩小或变换时用 cv::INTER_AREA 插值效果最好，能保留更多纹理细节，抗锯齿。

    return rotated_crop;
}

cv::Mat RemoveRedBaseSafely(const cv::Mat& rotated_crop)
{
    if(rotated_crop.empty())
        return rotated_crop;

    // 1. 克隆原图：坚持“非破坏性处理”
    cv::Mat result = rotated_crop.clone();
    cv::Vec3b track_bg_color(255, 255, 255);
    // ==================== 优化点：横扫前十行，寻找最亮的像素取平均作为背景色 ====================
    int scan_rows = std::min(10, rotated_crop.rows);
    std::vector<cv::Vec3b> top_pixels;
    
    for(int r=0;r<scan_rows;++r) {
        for(int c=0;c<rotated_crop.cols;++c) {
            top_pixels.push_back(rotated_crop.at<cv::Vec3b>(r,c));
        }
    }

    // 按照亮度 (B+G+R 的和) 从大到小排序，选出最亮的前 10% 像素点取平均
    if (!top_pixels.empty()) {
        std::sort(top_pixels.begin(),top_pixels.end(),[](const cv::Vec3b& a,const cv::Vec3b& b) {
            return (int)a[0]+a[1]+a[2]>(int)b[0]+b[1]+b[2];
        });

        int take_count=std::max(1,(int)(top_pixels.size()*0.1)); // 取最亮的前 10%
        long long sum_b=0,sum_g=0,sum_r=0;
        for(int i=0;i<take_count;++i) {
            sum_b+=top_pixels[i][0];
            sum_g+=top_pixels[i][1];
            sum_r+=top_pixels[i][2];
        }
        
        track_bg_color=cv::Vec3b(sum_b/take_count,sum_g/take_count,sum_r/take_count);
    }
    else {
        // 兜底退路
        track_bg_color=rotated_crop.at<cv::Vec3b>(2,rotated_crop.cols/2);
    }
    // ====================================================================================

    // 获取 HSV 红色掩码
    cv::Mat hsv,red_mask;
    cv::cvtColor(rotated_crop,hsv,cv::COLOR_BGR2HSV);
    get_red_mask(hsv,red_mask);

    // 3. 形态学闭运算与膨胀
    cv::Mat kernel=cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3));
    cv::morphologyEx(red_mask,red_mask,cv::MORPH_CLOSE,kernel);

    cv::Mat dilate_kernel=cv::getStructuringElement(cv::MORPH_RECT,cv::Size(5,5));
    cv::dilate(red_mask,red_mask,dilate_kernel);

    // 4. 连通域分析
    cv::Mat labels, stats, centroids;
    int num_labels=cv::connectedComponentsWithStats(red_mask,labels,stats,centroids,8);

    int safety_line = static_cast<int>(rotated_crop.rows * 0.45);

    for(int i = 1; i < num_labels; ++i) {
        int x = stats.at<int>(i, cv::CC_STAT_LEFT);
        int y = stats.at<int>(i, cv::CC_STAT_TOP);
        int w = stats.at<int>(i, cv::CC_STAT_WIDTH);
        int h = stats.at<int>(i, cv::CC_STAT_HEIGHT);
        int area = stats.at<int>(i, cv::CC_STAT_AREA);

        // 空间硬隔离
        if(y < safety_line) {
            continue; 
        }

        bool touches_bottom = (y + h > rotated_crop.rows * 0.75);

        // 提取当前连通域坐标以计算最小外接矩形
        std::vector<cv::Point> pts;
        for(int r = y; r < y + h; ++r) {
            for(int c = x; c < x + w; ++c) {
                if(r >= 0 && r < labels.rows && c >= 0 && c < labels.cols) {
                    if(labels.at<int>(r, c) == i) {
                        pts.push_back(cv::Point(c, r));
                    }
                }
            }
        }

        if(pts.empty()) continue;

        cv::RotatedRect min_rect = cv::minAreaRect(pts);
        float rect_w = min_rect.size.width;
        float rect_h = min_rect.size.height;
        
        float long_side = std::max(rect_w, rect_h);
        float short_side = std::min(rect_w, rect_h);
        float true_ratio = (short_side > 0) ? (long_side / short_side) : 0.0f;

        bool is_bottom_base = (true_ratio > 1.5f && touches_bottom);

        if(is_bottom_base) {
            // 精准抹除底座像素
            for(int r = y; r < y + h; ++r) {
                for(int c = x; c < x + w; ++c) {
                    if(r >= 0 && r < result.rows && c >= 0 && c < result.cols) {
                        if(labels.at<int>(r, c) == i) {
                            result.at<cv::Vec3b>(r, c) = track_bg_color;
                        }
                    }
                }
            }
        }
    }
    return result;
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


void get_pic(void)
{
    static int frame_count=0;
    //获取原始帧
    if(mid_line_detect()&&frame_count%10==0){
        cv::Rect found_roi=QuickDetect(target_trigger_x,target_trigger_y,raw_pic);
        if(found_roi.width>0&&found_roi.height>0){
            cv::Mat rotated = CropAndPerspective(raw_pic, current_rRect,trigger);
            if(rotated.empty())
                return;
            cout<<trigger<<endl;
            cv::Mat final_img = RemoveRedBaseSafely(rotated);
            if(final_img.empty()){
                return;
            }
            lyc_udp.udp_send_image(final_img);
            //cout<<frame_count<<endl;
            usleep(2000);
        }
    }
    frame_count++;
}

vector<Image_Type>vote;    //投票
bool begin_recognition=false;   //返回给控制端进行减速的标志位
bool bypass_action_flag=false;  //当trigger到达10时返回给控制端的绕行bool

//辅助函数：统计vector种出现次数最多的目标类型
Image_Type GetMajorityVote(const vector<Image_Type>& votes)
{
    if(votes.empty())
        return unknown;

    int count_supplies=0;
    int count_vehicle=0;
    int count_weapon=0;

    for(const auto&type:votes){
        if(type==supplies)
            count_supplies++;
        else if(type==vehicle)
            count_vehicle++;
        else if(type==weapon)
            count_weapon++;
    }

    if(count_supplies>=count_vehicle&&count_supplies>=count_weapon&&count_supplies>0)
        return supplies;
    if(count_supplies>=count_supplies&&count_vehicle>=count_weapon&&count_vehicle>0)
        return vehicle;
    if(count_weapon>=count_supplies&&count_weapon>=count_vehicle&&count_weapon>0)
        return weapon;

    return unknown;
}

/*
void Bypass_Recovery_Line(void)
{
    // 初始化左右边界搜索起点
    int x1=-1,x2=-1;
    // 动态确定扫描起始高度（核心关键点）
    // 因为刚绕过障碍物，近处有残影。我们直接利用最长白列的高度 Long_WhiteLine_Y 作为基础，
    // 把搜索起始行定在障碍物上方、视野干净的远端（例如 Long_WhiteLine_Y 往下放一点点，或者固定在一个安全的远端行）
    int start=starty-30; 
    int col_offset=40;
    // 以最长白列的 X 坐标为绝对核心，向左右两侧寻找黑白跳变点
    // 这样无论车身怎么歪，只要远端白列抓对了，左右边线就一定能精准抓到
    for(int row_offset=0;row_offset<30;row_offset+=5){
        int current_y=start-row_offset;
        if(current_y<10){
            break;
        }
        //向左找左边缘(x1初始何以设在靠中间偏右一点，或者直接从中心160向左扫)
        if(x1==-1){
            for(int i=(IMG_W>>1+col_offset);i>Block_Size;--i){
                if(AT(i,current_y>=OSTU_threshold&&AT(i-1,current_y)<OSTU_threshold)){
                    x1=i;
                    break;
                }
            }
        }
        //向右找右边缘(x2从中心160向右扫)
        if(x2==-1){
            for(int i=(IMG_W>>1-col_offset);i<IMG_W-Block_Size;++i){
                if(AT(i,current_y)>=OSTU_threshold&&AT(i+1,current_y)<OSTU_threshold){
                    x2=i;
                    break;
                }
            }
        }
        //如果左右至少找到了一条，就可以跳出外层行循环
        if(x1!=-1||x2!=-1){
            break;
        }
    }
    

    for(int i=Long_WhiteLine_X;i<IMG_W-Block_Size/2-1;++i){
        if(AT(i,start)>=OSTU_threshold&&AT(i+1,start)<OSTU_threshold){
            x2=i; // 右边缘
            break;
        }
    }
    //
    // 远端二次爬线与处理
    Far_Left_Num=Far_Right_Num=0;
    findline_lefthand_adaptive(x1,start,FarLeft_Line,&Far_Left_Num);//存在黑白跳变，寻找左线
    findline_righthand_adaptive(x2,start,FarRight_Line,&Far_Right_Num);//存在黑白跳变，寻找右线
    blur_points(FarLeft_Line,Far_Left_Num,FarBlueLeft_Line);
    blur_points(FarRight_Line,Far_Right_Num,FarBlueRight_Line);
    resample_points(FarBlueLeft_Line,Far_Left_Num,FarRepLeft_Line,&FarRepLnum);
    resample_points(FarBlueRight_Line,Far_Right_Num,FarRepRight_Line,&FarRepRnum);
    // 路径重建：平移生成远端中线
    track_leftline(FarRepLeft_Line,0,FarRepLnum,Left_Mid_Line,&L_Mid_Num);
    track_rightline(FarRepRight_Line,0,FarRepRnum,Right_Mid_Line,&R_Mid_Num);
    // 择优选择与补线
    // 如果你知道自己是向右绕过障碍物的，回正时可以适当偏向信任左侧或右侧
    if(L_Mid_Num>R_Mid_Num) {
        Cross_Path_Interpolation(Left_Mid_Line,&L_Mid_Num);
        LRType=MID_Left;
    } 
    else{
        Cross_Path_Interpolation(Right_Mid_Line,&R_Mid_Num);
        LRType=MID_Right;
    }
}*/

void target_detect_deal(void)
{
    int detection_result=mid_line_detect();   //尝试检测
    switch(current_bypass_state){
        // ------------------------------------------------
        // 状态0：正常检测与投票阶段
        // ------------------------------------------------
        case STATE_IDLE:{
            bypass_action_flag=false;//默认不触发绕行
            begin_recognition=false; //避免目标短暂丢失时状态残留
            if(trigger<45){
                current_bypass_state=STATE_DECEL_VOTE;
            }
            break;
        }
        // ------------------------------------------------
        // 状态1：减速+投票收集阶段(20<=trigger<=45)
        // ------------------------------------------------     
        case STATE_DECEL_VOTE:{
            //在此状态下，拉高减速标志位通知底层控制端减速
            begin_recognition=true;
            bypass_action_flag=false;

            //如果车子因为某些原因又退回去了(trigger>45)
            if(trigger>45){
                begin_recognition=false;
                vote.clear();
                current_bypass_state=STATE_IDLE;
                break;
            }
            //视觉检测与投票收集
            cv::Rect found_roi=QuickDetect(target_trigger_x,target_trigger_y,raw_pic);
            if(found_roi.width>0&&found_roi.height>0&&begin_recognition){
                cv::Mat rotated=CropAndPerspective(raw_pic,current_rRect,trigger);
                if(!rotated.empty()){
                    cv::Mat final_img=RemoveRedBaseSafely(rotated);
                    if(!final_img.empty()){
                        cv::resize(final_img,last_cropped_img_,cv::Size(96,96));
                        string result=ncnn_net_.Infer(last_cropped_img_);
                        Image_Type current_inferred_type=unknown;
                        if(result=="supplies")
                            current_inferred_type=supplies;
                        else if(result=="vehicle")
                            current_inferred_type=vehicle;
                        else if(result=="weapon")
                            current_inferred_type=weapon;
                        
                        vote.push_back(current_inferred_type);
                        cout<<"推理结果： "<<result<<" | 当前投票池大小: "<<vote.size()<<endl;
                    }
                }
            }
            //当trigger到达20的临界点时，结束投票，进行结算，并跃迁到盲行绕行状态
            if(trigger<=20){
                if(!vote.empty()&&vote.size()>8){
                    current_target=GetMajorityVote(vote);
                    cout << "【投票结算完成】最终选定目标类型: " << current_target << endl;
                    vote.clear();
                    //投票完成，正式进入盲行切出状态
                    current_bypass_state=STATE_BYPASS_BLIND;
                }
                else {
                    vote.clear();
                    current_target=unknown;
                }
            }
            break;
        }
        // ------------------------------------------------
        // 状态2：切出与盲行阶段(惯导+最长白列偏航角+动态PID)
        // ------------------------------------------------
        case STATE_BYPASS_BLIND:{
            std::cout<<1<<endl;
            bypass_action_flag=true;    //触发绕行动作
            begin_recognition=false;

            //结合最长白列偏航角与 current_target 传入惯导控制函数

            // 当编码器距离积分 或 陀螺仪角度积分达标后，进入视觉软着陆找线
            if (/* check_blind_distance_reached() */ true) { 
                current_bypass_state = STATE_VISION_RECOVERY;
            }
            break;
        }
        // ------------------------------------------------
        // 状态3：切出与盲行阶段(惯导+最长白列偏航角+动态PID)
        // ------------------------------------------------
        case STATE_VISION_RECOVERY: {
            std::cout<<2<<endl;
            bypass_action_flag = true;    
            Find_Line();
            //基础滤波与重采样
            blur_points(Left_Line, Left_Num, Blue_Left_Line);
            blur_points(Right_Line, Right_Num, Blue_Right_Line);
            resample_points(Blue_Left_Line, Left_Num, Rep_Left_Line, &Rep_Lnum);
            resample_points(Blue_Right_Line, Right_Num, Rep_Right_Line, &Rep_Rnum);
            bool flag_of_recover=Rep_Lnum>40||Rep_Rnum>40;
            //连续多帧稳定计数器
            static int stable_counter=0;
            const int required_stable_frames=5;  //连续8帧
            if(flag_of_recover){
                stable_counter++;
            }
            else{
                //如果中间有任何一帧拉跨，立刻清零重来，防止误判
                stable_counter=0;
            }
            if(stable_counter<required_stable_frames){
                if(Rep_Lnum>0)
                    track_leftline(Rep_Left_Line,0,Rep_Lnum,Left_Mid_Line,&L_Mid_Num);
                else
                    L_Mid_Num=0;
                if(Rep_Rnum>0)
                    track_rightline(Rep_Right_Line,0,Rep_Rnum,Right_Mid_Line,&R_Mid_Num);
                else    
                    R_Mid_Num=0;
            }
            // 视觉连续多帧稳定找到主赛道
            else{
                //稳定达标之后说明车已经回正
                local_angel_points(Rep_Left_Line,Rep_Lnum,Left_Angel);
                local_angel_points(Rep_Right_Line,Rep_Rnum,Right_Angel);
                nms_angle(Left_Angel,Rep_Lnum,Nms_Left_Angel);
                nms_angle(Right_Angel,Rep_Rnum,Nms_Right_Angel);
                find_corners();
                Speed_Change();
                //平移建模
                if(L_Out==LeftP.LP&&LeftP.ID<70)
                    track_leftline(Rep_Left_Line,0,LeftP.ID,Left_Mid_Line,&L_Mid_Num);
                else
                    track_leftline(Rep_Left_Line,0,Rep_Lnum,Left_Mid_Line,&L_Mid_Num);
                if(L_Out==RightP.LP&&RightP.ID<70)
                    track_rightline(Rep_Right_Line,0,RightP.ID,Right_Mid_Line,&R_Mid_Num);
                else
                    track_rightline(Rep_Right_Line,0,Rep_Rnum,Right_Mid_Line,&R_Mid_Num);

                //最终仲裁，准备交权
                Track_Choose();
                //状态正式切回IDLE,结束绕行
                stable_counter=0;     //清零，为下一次绕行做准备
                current_target = unknown;
                bypass_action_flag = false;
                current_bypass_state = STATE_IDLE; // 一切恢复正常，回到初始闲置态
            }
            break;
        }
    }
}


/*
void target_detect_deal(void)
{
    bypass_action_flag=false;   //每次进入先默认重置为0
    begin_recognition=false;    //避免目标短暂丢失时状态残留
    int detection_result=mid_line_detect();   //尝试检测
    if(trigger>45){
        begin_recognition=false;
    }
    //cout<<"trigger: "<<trigger<<endl;
    //投票收集阶段
    if(trigger>=20&&trigger<=45){
        cv::Rect found_roi=QuickDetect(target_trigger_x,target_trigger_y,raw_pic);
        if(found_roi.width>0&&found_roi.height>0){
            begin_recognition=true;
            //cout<<"begin_recognition: "<<begin_recognition<<endl;
            cv::Mat rotated=CropAndPerspective(raw_pic,current_rRect,trigger);
            if(!rotated.empty()){
                cv::Mat final_img=RemoveRedBaseSafely(rotated);
                if(!final_img.empty()){
                    //VisualizeDetection(show_mat,found_roi,angle,final_img.rows,rotated);
                    cv::resize(final_img,last_cropped_img_,cv::Size(96,96));
                    //执行推理结果并将结果压入投票箱
                    string result=ncnn_net_.Infer(last_cropped_img_);
                    Image_Type current_inferred_type=unknown;
                    if(result=="supplies")
                        current_inferred_type=supplies;
                    else if(result=="vehicle")
                        current_inferred_type=vehicle;
                    else if(result=="weapon")
                        current_inferred_type=weapon;

                    vote.push_back(current_inferred_type);
                    cout<<"推理结果: "<<result<<" | 当前投票池大小："<<vote.size()<<endl;
                }
            }
        }
    }

    //结算与绕行触发阶段：当trigger到达10时对投票箱进行结算
    if(trigger<=20){
        if(!vote.empty()){
            current_target=GetMajorityVote(vote);
            //cout << "【投票结算完成】最终选定目标类型: " << current_target << endl;
            //清空投票箱，为下一次识别做准备
            vote.clear();
        }
        else{
            current_target=unknown;
        }
        //触发绕行bool信号返回给控制端
        bypass_action_flag=true;
    }
}
*/

void track_trapezoid_bypass(float pts_in[][2],int start,int end,float pts_out[][2],int *num,int trigger_)
{
    *num=end-start;

    //定义梯形参数
    int ramp_len=15;    //转向过程长度(梯形斜边)
    int flat_len=20;    //保持偏移长度(梯形顶边)
    float max_bypass=30.0f;  //避让偏移量

    for(int i=start;i<end;i++){
        int safe_trigger=clip(trigger,start+ramp_len,end-1-flat_len-ramp_len);
        int dist=i-safe_trigger;
        float offset_val=0.0f;

        if(dist>=-ramp_len&&dist<0){
            //上升斜坡:由0升高至max
            offset_val=max_bypass*(float)(dist+ramp_len)/ramp_len;
        }
        else if(dist>=0&&dist<flat_len){
            //平台期：保持最大偏移
            offset_val=max_bypass;
        }
        else if(dist>=flat_len&&dist<flat_len+ramp_len){
            //下降斜坡
            offset_val=max_bypass*(1.0f-(float)(dist-flat_len)/ramp_len);
        }
        else{
            offset_val=0.0f;
        }

        //应用偏移量计算
        float total_offset=offset_val*img_k;
        float dx=pts_in[clip(i+APPROX_NUM,0,end-1)][0]-pts_in[clip(i-APPROX_NUM,0,end-1)][0];
        float dy=pts_in[clip(i+APPROX_NUM,0,end-1)][1]-pts_in[clip(i-APPROX_NUM,0,end-1)][1];
        float dn=fast_invsqrt(dx*dx+dy*dy);
        dx*=dn;
        dy*=dn;
        if(LRType==MID_Left){
            pts_out[i-start][0]=pts_in[i][0]-dy*total_offset;
            pts_out[i-start][1]=pts_in[i][1]+dx*total_offset;
        }
        else{
            pts_out[i-start][0]=pts_in[i][0]+dy*total_offset;
            pts_out[i-start][1]=pts_in[i][1]-dx*total_offset;
        }
    }
}