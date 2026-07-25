#include "Drive.hpp"
#include "Gyro.hpp"
#include "main.hpp"
#include "Tof.hpp"
#include "PID.hpp"
#include "lq_reg_gpio.hpp"
#include "lq_reg_atim_pwm.hpp"
#include "lq_reg_pwm_encoder.hpp"
#include "lq_reg_pwm.hpp"
#include "lq_reg_gtim_pwm.hpp"
#include <atomic>
#include <algorithm>

using namespace std;
static std::atomic<float> shared_angle_output{0.0f};   // 角度环输出差速量 -> 速度环输入
//static std::atomic<float> shared_diff_speed{0.0f};  

// 外部全局变量（在 main.cpp 中定义）
extern float base_speed;

//UDP发送的全局变量
float tar_L = 0.0f;
float tar_R = 0.0f;
float speed_L=0;
float speed_R=0;
float dif_speed=0;
float target_Omega=0;
float cur_Omega=0;
float error=0;
float DYNA_SPEED=0;
int L_PWM=0;
int R_PWM=0;

//车辆尺寸
// 陀螺仪 z 轴角速度 (rad/s) → 等效左右轮差速 (RPS) 的转换系数
// 理论公式：系数 = 轮距 × 减速比 / 车轮周长
//
// 推导：
//   线速度差 Δv = ω × 轮距  (mm/s)
//   轮子 RPS 差 = Δv / 车轮周长 = ω × 轮距 / 车轮周长
//   编码器 RPS 差 = 轮子 RPS 差 × 减速比 = ω × 轮距 × 减速比 / 车轮周长
//
// 参数：轮距 180mm，车轮直径 63mm，减速比 4
//   GYRO_TO_DIFF_RPS = 180 × 4 / (63×π) ≈ 3.64
// 请根据实际车轮间距、车轮直径和减速比标定此值

#define WHEEL_DIAMETER              63.0f //车轮直径 63mm
#define WHEEL_CIRCUMFERENCE         (WHEEL_DIAMETER * PI) // 轮子转一圈的物理距离(约188.5mm)

#define WHEEL_TRACK                 180.0f  // 轮距 (mm)
#define GEAR_RATIO                  4.0f    // 电机减速比（编码器 RPS : 轮子 RPS）
#define GYRO_TO_DIFF_RPS            (WHEEL_TRACK * GEAR_RATIO / WHEEL_CIRCUMFERENCE)


static double total_distance_mm = 0.0; //里程累计变量

// 目标速度（模块内部状态）
static float Target_Speed = 0.0f;


Motor Motor_L; //左轮电机
Motor Motor_R; //右轮电机

ls_gpio Right_Motor_GPIO(PIN_21,GPIO_MODE_OUT); // 右轮电机控制 GPIO
ls_gpio Left_Motor_GPIO(PIN_22,GPIO_MODE_OUT);  // 左轮电机控制 GPIO

ls_atim_pwm Right_Motor_PWM(ATIM_PWM0_PIN81, 10000, 0); // 右轮电机 PWM  //电机1
ls_atim_pwm Left_Motor_PWM(ATIM_PWM1_PIN82, 10000, 0);  // 左轮电机 PWM  //电机2

ls_encoder_pwm Encoder_R(ENC_PWM0_PIN64, PIN_72); // 右轮编码器
ls_encoder_pwm Encoder_L(ENC_PWM1_PIN65, PIN_73); // 左轮编码器

//无刷电机部分
// --- 定义电调的PWM频率 (手册支持 50Hz-300Hz) ---
#define ESC_PWM_FREQ            200 

// 占空比换算公式：1ms 对应的占空比数值 = 10 * 当前频率
#define ESC_INIT_DUTY           (10 * ESC_PWM_FREQ)

// 实例化无刷电调对象 
ls_pwm ESC_1(PWM3_PIN89, ESC_PWM_FREQ, ESC_INIT_DUTY);           // 无刷驱动1
ls_gtim_pwm ESC_2(GTIM_PWM3_PIN77, ESC_PWM_FREQ, ESC_INIT_DUTY); // 无刷驱动2

// 全局风扇目标油门百分比 
float Fan_Speed_Target =60.0f;

int ENC_Int=0;
int ENC_Start=0;
int tempEnc=0;

extern lq_udp_client syd_udp;
extern mpu_data_t mpu_data;
extern int run;
extern std::atomic<float> camera_error;
float K_turn = 0.0f;

/**************************************************************************
 * 函数功能：一阶低通滤波器（基于截止频率和采样时间）
 * 入口参数：hz   截止频率（Hz），决定滤波器允许通过的最高频率
 *           time 采样时间间隔（秒），即函数调用周期
 *           in   当前时刻的输入值（待滤波的原始信号）
 *           out  指向滤波结果的指针，同时作为输入（上次滤波值）和输出（本次滤波结果）
 * 返回  值：无
 * 说    明：传递函数模型 G(s) = 1/(τs+1)，其中 τ = 1/(2π·hz) 为时间常数
 *           离散化公式：y(k) = y(k-1) + α·[x(k) - y(k-1)]
 *           α = T/(T+τ) = 1/(1 + 1/(hz·2π·T))
 *           示例：LPF_1(20Hz, 5ms, raw_speed, &filtered_speed)
 *                 α ≈ 0.386，即新值权重 38.6%，历史值权重 61.4%
 **************************************************************************/
void LPF_1(float hz, float time, float in, float *out)
{
    float alpha = 1.0f / (1.0f + 1.0f / (hz * 6.28f * time));
    *out += alpha * (in - *out);
}


void Motor_Init(void)  //电机使能
{
    Left_Motor_PWM.atim_pwm_enable();
    Right_Motor_PWM.atim_pwm_enable();
}

void Fan_Init(void)  // 无刷风扇使能
{
    ESC_1.pwm_enable();
    ESC_2.gtim_pwm_enable();
}

// 设置风扇油门 (0.0 - 100.0 百分比)
// 设置风扇油门 (0.0 - 100.0 百分比)
void Set_Fan_Speed(float throttle_1, float throttle_2) 
{
    // 1. 安全限幅
    if (throttle_1 < 0.0f) throttle_1 = 0.0f;
    if (throttle_1 > 100.0f) throttle_1 = 100.0f;
    if (throttle_2 < 0.0f) throttle_2 = 0.0f;
    if (throttle_2 > 100.0f) throttle_2 = 100.0f;

    // 2. 根据当前设定的频率，动态计算 1ms 和 2ms 对应的占空比数值
    uint32_t duty_min = 10 * ESC_PWM_FREQ; // 1ms 对应的占空比 (最低油门)
    uint32_t duty_max = 20 * ESC_PWM_FREQ; // 2ms 对应的占空比 (最高油门)
    uint32_t duty_range = duty_max - duty_min;

    // 3. 百分比换算占空比 (0% -> duty_min, 100% -> duty_max)
    uint32_t duty_1 = duty_min + (uint32_t)((throttle_1 / 100.0f) * duty_range);
    uint32_t duty_2 = duty_min + (uint32_t)((throttle_2 / 100.0f) * duty_range);

    // 4. 输出给底层驱动
    ESC_1.pwm_set_duty(duty_1);
    ESC_2.gtim_pwm_set_duty(duty_2);
}
 
//左右轮 输出电平互反 <==驱动安装时左右轮的接的电源极性是反的
void Right_Motor_Set_Speed(int pwm) //设置右轮电机速度
{
    if (pwm >=0) //前进或者停止
    {
        Right_Motor_GPIO.gpio_level_set(GPIO_LOW); // 低电平，电机正转
        Right_Motor_PWM.atim_pwm_set_duty(pwm); // 占空比，控制速度
    }
    else //后退
    {
        Right_Motor_GPIO.gpio_level_set(GPIO_HIGH); // 高电平，电机反转
        Right_Motor_PWM.atim_pwm_set_duty(-pwm); // 占空比，控制速度
    }
}

void Left_Motor_Set_Speed(int pwm) //设置左轮电机速度
{
    if(pwm >=0) //前进或者停止
    {
        Left_Motor_GPIO.gpio_level_set(GPIO_HIGH); // 高电平，电机反转
        Left_Motor_PWM.atim_pwm_set_duty(pwm); // 占空比，控制速度
    }
        else //后退
    {
        Left_Motor_GPIO.gpio_level_set(GPIO_LOW); // 低电平，电机正转
        Left_Motor_PWM.atim_pwm_set_duty(-pwm); // 占空比，控制速度
    }
}


void Encoder_Speed_Get(float *Speed_L, float *Speed_R){
    // 读取原始转速 (RPS) 
    // 龙邱库 encoder_get_count() 返回的是 float 类型的转速（转/秒）
    // 左轮取反是因为左右轮物理安装方向相反
    float raw_L = -Encoder_L.encoder_get_count();
    float raw_R =  Encoder_R.encoder_get_count();
    Motor_L.Encoder_Speed = raw_L;
    Motor_R.Encoder_Speed = raw_R;
    //printf("左轮速度: %.2f  右轮速度: %.2f\n", *Speed_L, *Speed_R);

    // 左右轮独立速度滤波 (30Hz 截止频率, 5ms 采样周期) =====
    // α = 1 / (1 + 1/(30 × 6.28 × 0.005)) ≈ 0.484
    static float filter_L = 0.0f;
    static float filter_R = 0.0f;
    LPF_1(30.0f, 0.005f, raw_L, &filter_L);
    LPF_1(30.0f, 0.005f, raw_R, &filter_R);

    //里程累计
    float avg_speed_rps = (filter_L + filter_R) * 0.5f;
    float step_distance = avg_speed_rps * WHEEL_CIRCUMFERENCE * 0.005f;
    total_distance_mm += step_distance;
    ENC_Int = (int64_t)total_distance_mm;

    *Speed_L = filter_L;
    *Speed_R = filter_R;

}

void PWM_test()
{
    int pwm = 3000; // 设定一个较大的占空比值，测试电机响应
    Left_Motor_Set_Speed(pwm);
    Right_Motor_Set_Speed(pwm);
    float Speed_L, Speed_R;
    Encoder_Speed_Get(&Speed_L, &Speed_R);
    cout<<"pwm: "<<pwm<<endl;
    
    //char udp_buf[128];
    //snprintf(udp_buf, sizeof(udp_buf), "%f,%f\n", (float)Speed_L, (float)Speed_R);
    //syd_udp.udp_send_string(udp_buf);
    return;
}


void Set_Target_Speed(float speed)
{
    static const float Max_Speed = 100.0f;
    Target_Speed = std::max(0.0f, std::min(speed, Max_Speed));
}

float Get_TargetSpeed(void)
{
    return Target_Speed;
}



void Control_Init(float base_speed, 
                  float angle_Kp,  float angle_Kd, float angle_maxOut,
                  float Omega_Kp, float Omega_Kd, float Omega_maxOut,  
                  float speed_Kp, float speed_Ki, float speed_maxOut) {
    // 初始化 PID 参数
    AnglePID_Init(&Angle_PID, angle_Kp,  angle_Kd, angle_maxOut);
    OmegaPID_Init(&Omega_PID, Omega_Kp, Omega_Kd, Omega_maxOut);
    SpeedPID_Init(&Motor_PID_L, speed_Kp, speed_Ki, speed_maxOut);
    SpeedPID_Init(&Motor_PID_R, speed_Kp, speed_Ki, speed_maxOut);

    // 设置基础速度（参数名与全局变量同名，直接用赋值）
    ::base_speed = base_speed;
}


void Init(){
    //lq_tft18_drv_init(0);
    Motor_Init();
    Control_Init(
        0.0f,                              // base_speed (由文件加载覆盖)
        0.0f, 0.0f, 250.0f,          // 角度环: Kp,  Kd, MaxOut (°/s)
        0.0f, 0.0f, 50.0f,   // 角速度环: Kp, Kd,  MaxOut (位置式PI)
        0.0f, 0.0f, 8000.0f);          // 速度环: Kp, Ki, maxout
    gyro_init();
    //tof_init();
    Fan_Init();
}

void Set_Base_Speed(float speed) {
    base_speed = speed;
}

void Control_UpdateCameraError(float error) {
    camera_error.store(error, std::memory_order_release);
}


void control_callback() 
{
    if (!run) {
        Motor_PID_L.Output = 0;
        Motor_PID_R.Output = 0;
        Left_Motor_Set_Speed(0);
        Right_Motor_Set_Speed(0);
        Set_Fan_Speed(0.0f, 0.0f);
        return; 
    }
    
    static uint32_t tick = 0;
    tick++; // 定时器 5ms 进一次，计数加1

    //Set_Fan_Speed(Fan_Speed_Target, Fan_Speed_Target);


    static float target_omega = 0.0f;
    static float omega_correction = 0.0f;
    static float diff_speed = 0.0f;
    float current_err = camera_error.load(std::memory_order_acquire);

    float err = current_err;
    target_omega = AnglePID_Calc(&Angle_PID, -err); 
    target_Omega = target_omega;
    
    float current_gz = mpu_data.gz * 57.29578f; 
    cur_Omega = current_gz; 
    //diff_speed = OmegaPID_Calc(&Omega_PID, target_omega, current_gz);
    omega_correction = OmegaPID_Calc(&Omega_PID, target_omega, current_gz);
    diff_speed = omega_correction*0.06353;  //speed_adj (RPS) = ω_correction(°/s) × (3.64 / 57.29578)= ω_correction(°/s) × 0.06353
    dif_speed = diff_speed;

    // 【3. 动力分配与底层速度环】：5ms 执行一次
    float Speed_L, Speed_R;
    Encoder_Speed_Get(&Speed_L, &Speed_R);
    speed_L = Speed_L; 
    speed_R = Speed_R;

    float dynamic_base_speed = Get_TargetSpeed();
    DYNA_SPEED=dynamic_base_speed;
    //float dynamic_base_speed = 30.0f; // 使用全局基础速度作为动态基准速度
    //cout<<"dynamic_base_speed: "<<dynamic_base_speed<<endl;
    
    int target_L, target_R;
    if (diff_speed > 0) {
        target_R = dynamic_base_speed + diff_speed * K_turn;          // R 加 diff
        target_L = dynamic_base_speed - diff_speed * (2.0f - K_turn); // L 减 diff
    } else {
        target_R = dynamic_base_speed + diff_speed * (2.0f - K_turn); // R 加 diff
        target_L = dynamic_base_speed - diff_speed * K_turn;          // L 减 diff
    }

    //float dynamic_base_speed = Get_TargetSpeed(); 
    /*float abs_diff = fabs(diff_speed);

    if ( fabs(abs_diff) > 5.0f) {
        dynamic_base_speed = base_speed - (abs_diff * 0.5f); 
        if (dynamic_base_speed < 25.0f) {
            dynamic_base_speed =25.0f; 
        }
    }*/
    //int target_L = dynamic_base_speed - diff_speed;
    //int target_R = dynamic_base_speed + diff_speed;


    // 保存到全局变量供 UDP 发送
    tar_L = target_L;
    tar_R = target_R;



    int pwm_left = Left_SpeedPID(&Motor_PID_L, target_L, Speed_L);
    int pwm_right = Right_SpeedPID(&Motor_PID_R, target_R, Speed_R);
    L_PWM = pwm_left;
    R_PWM = pwm_right;


    if (pwm_left > Motor_PID_L.MaxOut) pwm_left = Motor_PID_L.MaxOut;
    if (pwm_left < -Motor_PID_L.MaxOut) pwm_left = -Motor_PID_L.MaxOut;
    if (pwm_right > Motor_PID_R.MaxOut) pwm_right = Motor_PID_R.MaxOut;
    if (pwm_right < -Motor_PID_R.MaxOut) pwm_right = -Motor_PID_R.MaxOut;


    Left_Motor_Set_Speed((int) pwm_left);
    Right_Motor_Set_Speed((int) pwm_right);
    
    //UDP_data_VOFA1(); 

    if (tick >= 10000) tick = 0;
}

void UDP_data_VOFA1(void) {
 // --- VOFA+ JustFloat 协议 (UDP单包发送) ---
    // 14通道float32(小端序) + 4字节帧尾 = 60字节, 一次udp_send发出
    float cam_err = camera_error.load(std::memory_order_acquire);

    float vofa_buf[11];
    vofa_buf[0]  = speed_L;
    vofa_buf[1]  = speed_R;
    vofa_buf[2]  = (float)tar_L;
    vofa_buf[3]  = (float)tar_R;
    vofa_buf[4]  = target_Omega;
    vofa_buf[5]  = cur_Omega;
    vofa_buf[6]  = DYNA_SPEED;
    vofa_buf[7]  = dif_speed;
    vofa_buf[8] = cam_err;
    vofa_buf[9] = (float)L_PWM;
    vofa_buf[10] = (float)R_PWM;

    // 拼包: 11*4字节float数据 + 4字节帧尾, 一次性UDP发送
    uint8_t send_buf[48];
    memcpy(send_buf, vofa_buf, 44);  // 11 × float
    send_buf[44] = 0x00; send_buf[45] = 0x00; // 帧尾
    send_buf[46] = 0x80; send_buf[47] = 0x7f; // 0x7f800000

    syd_udp.udp_send(send_buf, 48);
    return;
}



void UDP_data(void) {
    // --- 纯文本 UDP 数据包 ---
    // 发送为普通字符串，接收端直接按文本读取即可
    float cam_err = camera_error.load(std::memory_order_acquire);

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6)
        << speed_L << ","
        << speed_R << ","
        << (float)tar_L << ","
        << (float)tar_R << ","
        << target_Omega << ","
        << cur_Omega << ","
        << dif_speed << ","
        << DYNA_SPEED << ","
        << cam_err <<","
        << L_PWM << ","
        << R_PWM << "\n";

        //speed_L,speed_R,tar_L,tar_R,target_Omega,cur_Omega,dif_speed,Omega_correction,camera_error,L_PWM,R_PWM
    std::string text = oss.str();
    syd_udp.udp_send_string(text);
    return;
    
}

void Enc_Start(void)
{
    if(ENC_Start)return;
    ENC_Start=1;
    tempEnc=ENC_Int;
}

int64_t Enc_Get(void)
{
    if(0==ENC_Start)return 0;
    int64_t temp=(ENC_Int-tempEnc);
    return temp;
}

void Enc_Get_End(void)
{
    ENC_Start=0;
}

int64_t Get_IENC(void)
{
   return ENC_Int;
}