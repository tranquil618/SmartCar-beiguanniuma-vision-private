#include "Recognition.hpp"
#include "main.hpp"
#include <algorithm>
#include <iostream>
#include <vector>
#include <string>
#include <cmath>

using namespace std;

// 初始化全局变量
Image_Type TargetType = Image_Type::unknown;
priority_LR Prior_LRType = priority_LR::MID_Center; // 初始化绕行优先方向为左侧
extern int run; // 从 main.cpp 引入全局运行状态变量
bool flag = false; // 初始化全局标志变量
bool flag_stop=false;// 用于指示是否停止识别线程
Image_Type winner = Image_Type::unknown; // 初始化获胜类别
float error_offset=0.0f; // 初始化误差补偿值

TargetDetector::TargetDetector() : model_initialized_(false) {
    // 1. 初始化红色 HSV 阈值
    lower_red1_ = cv::Scalar(0, 70, 45);
    upper_red1_ = cv::Scalar(10, 255, 255);
    lower_red2_ = cv::Scalar(170, 70, 45);
    upper_red2_ = cv::Scalar(180, 255, 255);

    // 2. 初始化白色赛道 HSV 阈值
    lower_white_ = cv::Scalar(0, 0, 160);
    upper_white_ = cv::Scalar(180, 35, 255);

    // 3. 初始化裁剪外扩比例
    top_expand_ = 2.2;
    side_expand_ = 0.2;

    // 4. 构造函数中提前创建形态学核（降低运行期 CPU 消耗）
    k_white_close_ = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
    k_lane_dilate_ = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(25, 25));
    k_red_open_    = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

    // 5. 初始化 NCNN 模型参数
    std::string model_param = "tiny_classifier_fp32.ncnn.param";
    std::string model_bin   = "tiny_classifier_fp32.ncnn.bin";
    input_width_    = 96;
    input_height_   = 96;
    std::vector<std::string> labels = {"supplies", "vehicle", "weapon"};
    float mean_vals[3] = {123.675f, 116.28f, 103.53f};
    float norm_vals[3] = {0.01712475f, 0.017507f, 0.01742919f};

    ncnn_net_.SetModelPath(model_param, model_bin);
    ncnn_net_.SetInputSize(input_width_, input_height_);
    ncnn_net_.SetLabels(labels);
    ncnn_net_.SetNormalize(mean_vals, norm_vals);
    /*
    cout << "正在初始化NCNN模型..." << endl;
    if (ncnn_net_.Init()) {
        cout << "模型加载成功！" << endl;
        model_initialized_ = true;
    } else {
        cout << "警告：模型加载失败！将无法正常识别类别。" << endl;
    }*/
}

TargetDetector::~TargetDetector() {}


bool TargetDetector::processFrame(cv::Mat& input_frame, const std::string& save_path) {
    if (input_frame.empty()) {
        return false;
    }

    // 1. 静态 ROI 裁剪（将硬裁剪由 35% 放宽至 20%，防止坡道及远端靶板被切头）
    int roi_top = input_frame.rows*0.34f; 
    int roi_bottom=input_frame.rows*0.10f; // 额外保留底部 5% 区域，防止近距离目标被裁掉
    // 确保计算出的高度合法
    int roi_height = input_frame.rows - roi_top - roi_bottom;

    cv::Rect static_roi(0, roi_top, input_frame.cols, roi_height);
    cv::Mat roi_frame = input_frame(static_roi);

    // 2. 1/2 降采样提高速度（全面复用带 _ 的类成员变量）
    double scale = 0.5; 
    cv::resize(roi_frame, small_frame_, cv::Size(), scale, scale, cv::INTER_NEAREST);
    cv::cvtColor(small_frame_, hsv_, cv::COLOR_BGR2HSV);

    // 3. 提取白色赛道安全区
    cv::inRange(hsv_, lower_white_, upper_white_, white_mask_);
    cv::morphologyEx(white_mask_, white_mask_, cv::MORPH_CLOSE, k_white_close_);

    std::vector<std::vector<cv::Point>> lane_contours;
    cv::findContours(white_mask_, lane_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    // 安全复用 lane_mask_ 空间
    if (lane_mask_.size() != white_mask_.size() || lane_mask_.type() != CV_8UC1) {
        lane_mask_ = cv::Mat::zeros(white_mask_.size(), CV_8UC1);
    } else {
        lane_mask_.setTo(0);
    }

    bool has_lane = false;
    if (!lane_contours.empty()) {
        double max_lane_area = -1;
        int max_lane_idx = -1;
        for (size_t i = 0; i < lane_contours.size(); ++i) {
            double a = cv::contourArea(lane_contours[i]);
            if (a > max_lane_area) { max_lane_area = a; max_lane_idx = i; }
        }
        if (max_lane_idx != -1 && max_lane_area > (hsv_.rows * hsv_.cols * 0.02)) { // 对齐 Python 2% 赛道面积基准
            cv::drawContours(lane_mask_, lane_contours, max_lane_idx, cv::Scalar(255), -1);
            has_lane = true;
        }
    }
    
    // 远端红条膨胀保护区
    cv::dilate(lane_mask_, safe_lane_mask_, k_lane_dilate_);

    // 4. 提取红色双段 HSV 区域
    cv::inRange(hsv_, lower_red1_, upper_red1_, mask1_);
    cv::inRange(hsv_, lower_red2_, upper_red2_, mask2_);
    cv::bitwise_or(mask1_, mask2_, red_mask_);
    cv::morphologyEx(red_mask_, red_mask_, cv::MORPH_OPEN, k_red_open_);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(red_mask_, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // 5. 多维智能评分系统
    cv::Rect best_small_rect;
    double max_score = -1.0;
    bool found_any = false;

    double min_area = 10.0;          
    double max_area = 12500.0;
    double aspect_ratio_min = 1.4;
    double aspect_ratio_max = 5.0;
    double min_fill_ratio = 0.40;

    // 提前计算三角形防御所需的图像尺寸常量（基于降采样后的画布）
    int h = hsv_.rows;
    int w = hsv_.cols;
    double tri_w = w * 0.30;       // 对应 Python 30% 宽
    double tri_h = h * 0.60;       // 对应 Python 60% 高

    for (const auto& cnt : contours) {
        double area = cv::contourArea(cnt);
        if (area < 1.0) { 
            cv::Rect r = cv::boundingRect(cnt);
            area = r.width * r.height;
        }

        if (area >= min_area && area < max_area) { 
            cv::Rect rect = cv::boundingRect(cnt);

            // 【吸收优点 1】等效 Python 远端两侧三角形掩码（零耗时空间解析判定）
            int cx = rect.x + rect.width / 2;
            int cy = rect.y + rect.height / 2;
            
            // 左上角三角形区域判定: y / x < (tri_h / tri_w)
            if (cx < tri_w && cy < tri_h) {
                if (static_cast<double>(cy) < (tri_h / tri_w) * (tri_w - cx)) {
                    continue; // 处于左上角死角，直接剔除背景红光
                }
            }
            // 右上角三角形区域判定
            if (cx > (w - tri_w) && cy < tri_h) {
                if (static_cast<double>(cy) < (tri_h / tri_w) * (cx - (w - tri_w))) {
                    continue; // 处于右上角死角，直接剔除背景红光
                }
            }

            // 阶梯式填满度约束（吸收优点 2：放宽至 0.60 提高远距离鲁棒性，防边缘畸变致使漏检）
            double fill_ratio = area / static_cast<double>(rect.width * rect.height);
            if (area > 50.0 && fill_ratio < 0.60) 
                continue; 
            if (fill_ratio < min_fill_ratio) 
                continue;

            // 位置约束与中轴线偏差
            bool is_inside_lane = true;
            double lane_center_dist = 0.0;
            int center_x = std::max(0, std::min(safe_lane_mask_.cols - 1, cx));
            int center_y = std::max(0, std::min(safe_lane_mask_.rows - 1, cy));

            if (has_lane) {
                // 高速行指针扫描
                const uchar* row_ptr = safe_lane_mask_.ptr<uchar>(center_y);
                if (row_ptr[center_x] != 255) is_inside_lane = false;

                int left_bound = -1, right_bound = -1;
                for (int col = 0; col < safe_lane_mask_.cols; ++col) {
                    if (row_ptr[col] == 255) {
                        if (left_bound == -1) left_bound = col;
                        right_bound = col;
                    }
                }
                if (left_bound != -1) {
                    double lane_mid_x = (left_bound + right_bound) / 2.0;
                    lane_center_dist = std::abs(center_x - lane_mid_x);
                }
            } else {
                // 如果当前帧没识别到车道线，默认不进行车道线硬卡剔除，交由评分判定（增强视野丢失时的防御力）
                is_inside_lane = true; 
            }

            // 如果当前识别到了车道线，但目标完全不在安全区内，直接过滤
            if (has_lane && !is_inside_lane) {
                continue;
            }

            double aspect_ratio = static_cast<double>(rect.width) / rect.height;
            if (aspect_ratio > aspect_ratio_min && aspect_ratio < aspect_ratio_max) {
                
                // 复合评分机制
                double score = area;
                if (is_inside_lane) score += 10000.0; 
                
                // 【吸收优点 3】对齐 Python 严厉的长宽比偏离惩罚（惩罚系数由 20.0 提升至 100.0）
                score -= std::abs(aspect_ratio - 2.5) * 100.0; 
                score -= lane_center_dist * 5.0;

                if (score > max_score) {
                    max_score = score;
                    best_small_rect = rect;
                    found_any = true;
                }
            }
        }
    }

    if (!found_any) return false;

    // 6. 还原回原始坐标系
    int orig_rect_x = best_small_rect.x / scale;
    int orig_rect_y = (best_small_rect.y / scale) + roi_top;
    int orig_rect_w = best_small_rect.width / scale;
    int orig_rect_h = best_small_rect.height / scale;

    // 严格双向越界安全保护
    int y1 = std::max(0, static_cast<int>(orig_rect_y - orig_rect_h * top_expand_));
    int y2 = std::min(input_frame.rows, static_cast<int>(orig_rect_y + orig_rect_h));
    // 【最终化简】任它怎么粘连，底边直接锁死在红块上沿，红色绝对进不来
    //int y1 = std::max(0, static_cast<int>(orig_rect_y - orig_rect_h * top_expand_));
    //int y2 = std::min(input_frame.rows, static_cast<int>(orig_rect_y));

    int x1 = std::max(0, static_cast<int>(orig_rect_x - orig_rect_w * side_expand_));
    int x2 = std::min(input_frame.cols, static_cast<int>(orig_rect_x + orig_rect_w * (1.0 + side_expand_)));

    int crop_w = x2 - x1;
    int crop_h = y2 - y1;

    if (crop_w > 0 && crop_h > 0) {
        cv::Rect crop_roi(x1, y1, crop_w, crop_h);
        cv::Mat raw_crop = input_frame(crop_roi);

        // 7. 严格等比例缩放 + 居中补纯黑边扩展至标准 96x96
        int target_size = 96;
        double resize_scale = std::min(static_cast<double>(target_size) / raw_crop.cols, static_cast<double>(target_size) / raw_crop.rows);
        int nw = static_cast<int>(raw_crop.cols * resize_scale);
        int nh = static_cast<int>(raw_crop.rows * resize_scale);

        cv::Mat resized_img;
        cv::resize(raw_crop, resized_img, cv::Size(nw, nh), 0, 0, cv::INTER_LINEAR);
        int top_pad    = (target_size - nh) / 2;
        int bottom_pad = target_size - nh - top_pad;
        int left_pad   = (target_size - nw) / 2;
        int right_pad  = target_size - nw - left_pad;

        cv::copyMakeBorder(resized_img, last_cropped_img_, top_pad, bottom_pad, left_pad, right_pad, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
        
        if (!save_path.empty()) {
            cv::imwrite(save_path, last_cropped_img_);
        }

        // 安全绘图反馈
        cv::rectangle(input_frame, crop_roi, cv::Scalar(0, 255, 0), 2);
        cv::rectangle(input_frame, cv::Rect(orig_rect_x, orig_rect_y, orig_rect_w, orig_rect_h), cv::Scalar(0, 0, 255), 2);
        return true;
    }

    return false;
}

cv::Mat TargetDetector::getCroppedImage() const {
    return last_cropped_img_;
}

void TargetDetector::getTargetType(const cv::Mat& cropped_img) {
    if(cropped_img.empty() || !model_initialized_) {
        TargetType = Image_Type::unknown;
        return;
    }

    // 直接高速模型推理
    string result = ncnn_net_.Infer(cropped_img);

    if (result == "supplies") {
        TargetType = Image_Type::supplies;
    } 
    else if (result == "vehicle") {
        TargetType = Image_Type::vehicle;
    } 
    else if (result == "weapon") {
        TargetType = Image_Type::weapon;
    }
    else {
        TargetType = Image_Type::unknown;
    }
    cout << "推理结果: " << result << endl;
}


void TargetDetector::TriggerBackupStrategy(Image_Type type) 
{
    switch(type) {
        case Image_Type::supplies:
            cout << "[决策] 10帧判定为 医疗包 -> 触发向右绕行！保持 " << hold_duration_ms_ << "ms" << endl;
            //Dynamic_Mid_Offset = 2.0f;       // 举例：强行给中线叠加向右的偏移量
            //error_offset=5.0f;
            current_strategy_ = StrategyState::BYPASS_RIGHT;
            strategy_start_time_ = std::chrono::steady_clock::now();
            break;

        case Image_Type::weapon:
            cout << "[决策] 10帧判定为 武器包 -> 触发向左绕行！保持 " << hold_duration_ms_ << "ms" << endl;
            //Dynamic_Mid_Offset = 2.0f;      // 举例：强行给中线叠加向左的偏移量
            //error_offset=-5.0f;
            current_strategy_ = StrategyState::BYPASS_LEFT;
            strategy_start_time_ = std::chrono::steady_clock::now();
            break;

        case Image_Type::vehicle:
            cout << "[决策] 10帧判定为 车辆 -> 维持正中行驶。" << endl;
            //Dynamic_Mid_Offset=MID_DIS; // 直接把偏移量设置成默认的中线距离，强行回归正中
            //error_offset=0.0f;
            current_strategy_ = StrategyState::TRACK_MID;
            // 车辆不需要特殊绕行，维持原有常规巡线即可
            break;
        case Image_Type::unknown:
            cout << "[决策] 10帧判定为 未知 -> 维持正中行驶。" << endl;
            Dynamic_Mid_Offset=MID_DIS; // 直接把偏移量设置成默认的中线距离，强行回归正中
            error_offset=0.0f;
            current_strategy_ = StrategyState::TRACK_MID;
            // 对于未知类别，我们选择不改变当前策略，继续观察后续帧的投票结果
            break;
        default:
            break;
    }
}

void TargetDetector::UpdateStrategyStatus() 
{
    if (current_strategy_ == StrategyState::TRACK_MID) {
        return; // 处于普通巡线状态，无需处理超时
    }

    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - strategy_start_time_).count();

    if (elapsed >= hold_duration_ms_) {
        cout << "[状态机] ⏳ 绕行时间已满，平滑回归常规中线行驶。" << endl;
        
        // 恢复你的常规巡线控制变量
        //Track_Choose(); // 假设你有默认正常状态，请根据实际业务修改
        Dynamic_Mid_Offset = MID_DIS; // 恢复默认中线偏移量
        error_offset=0.0f;
        flag_stop = false; // 重置全局标志位，准备下一轮识别
        winner = Image_Type::unknown; // 重置投票结果
        current_strategy_ = StrategyState::TRACK_MID;
    }
}


// ================= 单线程串行集成调用函数 =================
void TargetDetector::ExecuteSynchronousDetection(const cv::Mat& current_frame) 
{
    // 1. 如果小车没有处于运行状态，直接跳过
    // (请确保 run 变量在作用域内可访问，如果是全局变量建议加上 extern 或改为类成员)
    if (!run) {
        return;
    }
    // 2. 状态机管理：如果当前正在执行绕行，判断是否超时
    UpdateStrategyStatus();
    if (current_strategy_ == StrategyState::BYPASS_RIGHT || current_strategy_ == StrategyState::BYPASS_LEFT) {
        return; 
    }
    // 3. 只有在常规巡线状态下，才去探索新目标（防止绕行中途被误检覆盖）
    if (current_strategy_ != StrategyState::TRACK_MID) {
        return; 
    }
    if (current_frame.empty()) {
        return;
    }
    // 【优化点】：因为 processFrame 内部要安全绘图并提交上位机显示，
    // 我们在此处传入非 const 的可变矩阵。如果后续主流程还要用原图，可以用 clone()
    cv::Mat editable_frame = current_frame.clone();
    // 4. 【前置哨兵】先通过传统视觉算法寻找红色靶板区域并裁剪
    flag = processFrame(editable_frame, "1.jpg"); 
    Image_Type current_frame_type = Image_Type::unknown;
    if (flag) {
        // 找到了目标！此时才触发高负载的 NCNN 模型推理
        getTargetType(last_cropped_img_);
        current_frame_type = TargetType; // 获取推理出的类别 (supplies / weapon / vehicle)
    } 
    else {
        // 没找到红色靶板，当前帧视为 unknown
        current_frame_type = Image_Type::unknown;
    }
    // 5. 将当前帧结果压入滑动窗口队列进行滤波
    frame_window_.push_back(current_frame_type);
    // 保持 5 帧滑动窗口大小
    if (frame_window_.size() > 3) {
        frame_window_.erase(frame_window_.begin());
    }
    /*
    // 当攒满 5 帧数据时进行安全投票判定
    if (frame_window_.size() == 5) {
        std::map<Image_Type, int> vote_map;
        for (auto t : frame_window_) {
            vote_map[t]++;
        }

        // 寻找得票最高的类别
        int max_votes = 0;
        Image_Type local_winner = Image_Type::unknown;
        
        for (auto const& [type, count] : vote_map) {
            if (count > max_votes) {
                max_votes = count;
                local_winner = type;
            }
        }

        // 设定决定性阈值：5 帧中某一特定有效类别必须达到 3 帧以上才触发
        if (max_votes >= 3 && local_winner != Image_Type::unknown) {
            winner = local_winner;                 // 更新全局投票结果
            flag_stop = true;                      // 设置全局标志位通知主线程停止当前识别循环
            TriggerBackupStrategy(local_winner);   // 根据投票结果触发对应的绕行策略
            frame_window_.clear();                 // 触发成功后清空窗口，为下一次巡线做准备
            return;
        } 
        // 【新增防御逻辑】：如果最高票是 unknown 且过半数，说明视野里确实没靶板
        // 及时清空窗口，防止历史残余的个别误检噪点在窗口中“借尸还魂”累计激发现象
        else if (local_winner == Image_Type::unknown && max_votes >= 3) {
            frame_window_.clear();
        }
    }*/
    // 当攒满 5 帧数据时进行【加权】投票判定
    if (frame_window_.size() == 3) {
        // 使用 double 或 int 存储每个类别的累计权重得分
        std::map<Image_Type, int> weight_vote_map;
        
        // 定义权重模板，索引越靠后（最新、最近），权重越大
        const std::vector<int> weights = {/*1, 1, 1, 1,*/ 1, 1, 1/*, 2, 3, 4*/}; // 10帧窗口，前6帧权重1，后4帧权重递增到4（最新一帧权重最高）

        // 1. 加权统计
        for (size_t i = 0; i < frame_window_.size(); ++i) {
            Image_Type t = frame_window_[i];
            weight_vote_map[t] += weights[i]; // 不再是++，而是加上对应的权重
        }

        // 2. 寻找权重得分最高的类别
        int max_weight_scores = 0;
        Image_Type local_winner = Image_Type::unknown;
        
        for (auto const& [type, score] : weight_vote_map) {
            if (score > max_weight_scores) {
                max_weight_scores = score;
                local_winner = type;
            }
        }

        // 3. 设定决定性权重阈值（总分8分，达到5分及以上触发，且胜者不能是 unknown）
        if (max_weight_scores >= 2 && local_winner != Image_Type::unknown) {
            winner = local_winner; 
            flag_stop = true;                      
            TriggerBackupStrategy(local_winner);   
            frame_window_.clear();                 
            return;
        } 
        // 防御逻辑：如果最高分是 unknown 且占绝对优势（比如 >= 5分），及时清空窗口
        else if (local_winner == Image_Type::unknown && max_weight_scores >= 2) {
            frame_window_.clear();
        }
    }
}