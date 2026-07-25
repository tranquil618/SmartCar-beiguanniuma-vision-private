/*
*Recognition.h
*
*Created on: 2026年4月3号
*  Author：LYC
*/
#ifndef CODE_RECOGNITION_H_
#define CODE_RECOGNITION_H_
#include <vector>
#include "Image.h"
#include "Cross.h"
#include <opencv2/core/mat.hpp> // 显式包含cv::Mat头文件（避免未定义）

//常量
//初始化红色HSV阈值
const cv::Scalar lower_red1_ = cv::Scalar(0, 70, 45);
const cv::Scalar upper_red1_ = cv::Scalar(10, 255, 255);
const cv::Scalar lower_red2_ = cv::Scalar(170, 70, 45);
const cv::Scalar upper_red2_ = cv::Scalar(180, 255, 255);
// 初始化白色赛道 HSV 阈值
const cv::Scalar lower_white_ = cv::Scalar(0, 0, 160);
const cv::Scalar upper_white_ = cv::Scalar(180, 35, 255);

//变量
extern int target_trigger_y;
extern int target_trigger_x;

extern cv::Mat hsv_img;    //HSV图像
extern cv::Mat red_mask;   //红色mask
extern cv::Mat target_crop;//裁剪结果

extern bool target_detect_flag;   //当前是否进入目标检测流程

//检测当前状态
enum BypassState{
    STATE_IDLE=0,         //未触发绕行，正常检测/投票
    STATE_DECEL_VOTE,     //减速+投票
    STATE_BYPASS_BLIND,   //切出与盲行阶段(惯导+最长白列偏航角)
    STATE_VISION_RECOVERY //视觉找回线
};
extern BypassState current_bypass_state;
//NCNN结果举例
enum Image_Type{
    unknown,
    supplies,
    vehicle,
    weapon
};
extern Image_Type current_target;
//切线斜率
extern float angle;
//中线跳变数组位置
extern int trigger;
//红色掩码，最终裁剪图片
extern cv::Mat mask1_, mask2_, red_mask_, last_cropped_img_;
//框选
extern cv::RotatedRect current_rRect;
extern cv::RotatedRect final_refind_rRect;
//投票
extern std::vector<Image_Type>vote;   
extern bool begin_recognition;
extern bool bypass_action_flag;
//函数部分
int mid_line_detect(void);    //中线跳变点监视
//专门提取红色掩码
void get_red_mask(cv::Mat&hsv_input,cv::Mat&output_mask);
//寻找红色标志位
cv::Rect QuickDetect(int center_x,int center_y,const cv::Mat&hsv_img);
//旋转与变换
cv::Mat CropAndPerspective(const cv::Mat& src, cv::RotatedRect rRect, int y_idx);
//掩盖红色标志位
cv::Mat RemoveRedBaseSafely(const cv::Mat& rotated_crop);
//可视化
void VisualizeDetection(cv::Mat& img, cv::Rect roi, float angle, int bottom_edge, cv::Mat& rotated_img);
//上传拍到的图片
void get_pic(void);
//辅助函数：统计vector种出现次数最多的目标类型
Image_Type GetMajorityVote(const std::vector<Image_Type>& votes);
//最终处理集成
void target_detect_deal(void);
//中线梯形偏移
void track_trapezoid_bypass(float pts_in[][2],int start,int end,float pts_out[][2],int *num,int trigger_);

#endif  //RECOGNITION_H