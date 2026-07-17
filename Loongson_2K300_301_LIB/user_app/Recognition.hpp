#ifndef TARGET_DETECTOR_HPP
#define TARGET_DETECTOR_HPP

#include <opencv2/opencv.hpp>
#include "Image.h"
#include "lq_ncnn.hpp"
#include <thread>
#include <mutex>
#include <atomic>
#include <string>
#include <chrono>
#include <vector>

typedef enum {
    supplies,
    vehicle,
    weapon,
    unknown
} Image_Type;

typedef enum {
    MID_Left_priority,
    MID_Right_priority,
    MID_Center
} priority_LR;

extern priority_LR Prior_LRType; // 声明全局变量，指示当前的绕行方向
extern Image_Type TargetType;
extern bool flag; // 用于指示是否成功找到目标并裁剪
extern bool flag_stop; // 用于指示是否停止识别线程
extern Image_Type winner; // 用于存储当前投票结果的获胜类别
extern float error_offset; // 用于存储当前的误差补偿值

class TargetDetector {
public:
    // 构造函数：初始化所有阈值并加载NCNN模型
    TargetDetector();
    ~TargetDetector();  
    
    // 核心接口：修改为可变引用，移除头文件中的非法类名前缀
    bool processFrame(cv::Mat& input_frame, const std::string& save_path = "1.jpg");

    // 获取最近一次成功裁剪的图像
    cv::Mat getCroppedImage() const;

    // 识别目标类型
    void getTargetType(const cv::Mat& cropped_img); 
    void React_Choose(const cv::Mat& input_frame);
    void StartThread();
    void StopThread();
    void UpdateFrame(const cv::Mat& frame);

    void ExecuteSynchronousDetection(const cv::Mat& current_frame);
    Image_Type current_winner = Image_Type::unknown; // 局部的、每帧都变的

private:
    // 线程内部分支函数
    void ThreadLoop();
    void ExecuteDetectionAndVote(const cv::Mat& local_frame);
    void TriggerBackupStrategy(Image_Type type);
    void UpdateStrategyStatus();

    // 线程与同步控制
    std::thread worker_thread_;
    std::atomic<bool> is_running_{false};
    std::mutex frame_mutex_;
    cv::Mat input_share_frame_;  // 主线程共享给识别线程的图像

    // 10帧滑动窗口
    std::vector<Image_Type> frame_window_;

    // 绕行状态机
    enum class StrategyState { TRACK_MID, BYPASS_LEFT, BYPASS_RIGHT };
    StrategyState current_strategy_;
    std::chrono::steady_clock::time_point strategy_start_time_;
    int hold_duration_ms_=1500; // 状态保持毫秒数

    // 基础配置参数
    cv::Scalar lower_white_;
    cv::Scalar upper_white_;
    cv::Scalar lower_red1_;
    cv::Scalar upper_red1_;
    cv::Scalar lower_red2_;
    cv::Scalar upper_red2_;
    
    double top_expand_;
    double side_expand_;

    // 预创建的形态学核（避免每帧 getStructuringElement）
    cv::Mat k_white_close_;
    cv::Mat k_lane_dilate_;
    cv::Mat k_red_open_;

    // 完美的 CPU 内存复用缓冲区（已移除重复定义的 last_cropped_img_）
    cv::Mat small_frame_, hsv_, white_mask_, lane_mask_, safe_lane_mask_;
    cv::Mat mask1_, mask2_, red_mask_, last_cropped_img_;

    // NCNN 模型相关组件
    LQ_NCNN ncnn_net_;
    bool model_initialized_;
    int input_width_;
    int input_height_;
};

#endif // TARGET_DETECTOR_HPP