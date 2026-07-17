#include "Drive.hpp"
#include "Recognition.hpp"
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
extern float conf_MAXL;      //左边线角点置信度，单位为0-1，越大表示越有可能是角点
extern float conf_MAXR;      //右边线角点置信度，单位为0-1，越大表示越有可能是角点
extern bool flag_stop;      // 用于指示是否停止识别线程
extern bool flag;           // 用于指示是否开始识别线程
float tar_L = 0.0f;
float tar_R = 0.0f;

float speed_L=0;
float speed_R=0;
float dif_speed=0;
float target_Omega=0;
float cur_gz=0;
float error=0;
float dyna_speed=0;

int L_PWM=0;
int R_PWM=0;

//float DA1A_Dis=0;
//float DL1A_Dis_Diff=0;

#define WHEEL_DIAMETER 63.0f //车轮直径 63mm
#define PI 3.1415926535f
#define WHEEL_CIRCUMFERENCE (WHEEL_DIAMETER * PI) // 轮子转一圈的物理距离(约188.5mm)


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


#define ESC_PWM_FREQ 300

#define ESC_INIT_DUTY (10 * ESC_PWM_FREQ)

// 实例化无刷电调对象 (初始状态为1ms最低油门，用于上电解锁)
ls_pwm ESC_1(PWM3_PIN89, ESC_PWM_FREQ, ESC_INIT_DUTY);           // 无刷驱动1
ls_gtim_pwm ESC_2(GTIM_PWM3_PIN77, ESC_PWM_FREQ, ESC_INIT_DUTY); // 无刷驱动2

// 全局风扇目标油门百分比 (0.0 到 100.0)
float Fan_Speed_Target =80.0f;

int ENC_Int=0;
int ENC_Start=0;
int tempEnc=0;

extern lq_udp_client syd_udp;
extern mpu_data_t mpu_data;
extern int run;
extern std::atomic<float> camera_error;

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
    Motor_L.Encoder_Speed = - Encoder_L.encoder_get_count() ;
    Motor_R.Encoder_Speed = Encoder_R.encoder_get_count() ;
    *Speed_L = Motor_L.Encoder_Speed;
    *Speed_R = Motor_R.Encoder_Speed;
    //printf("左轮速度: %.2f  右轮速度: %.2f\n", *Speed_L, *Speed_R);

   float arg_speed_rps = (Motor_L.Encoder_Speed + Motor_R.Encoder_Speed) / 2.0f;
   float dt = 0.005f;
   float step_distance = arg_speed_rps * WHEEL_CIRCUMFERENCE * dt;
   total_distance_mm += step_distance;
   total_distance_mm += step_distance;
   ENC_Int = (int64_t)total_distance_mm;
}

void kaihuan()
{
    int pwm = 3000; // 设定一个较大的占空比值，测试电机响应
    Left_Motor_Set_Speed(pwm);
    Right_Motor_Set_Speed(pwm);
    float Speed_L, Speed_R;
    Encoder_Speed_Get(&Speed_L, &Speed_R);
    cout<<"pwm: "<<pwm<<endl;
    cout<<"speed_L: "<<Speed_L<<"   speed_R: "<<Speed_R<<endl;
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
    //Control_Init(0, 0.0f,0.0f, 300.0f, 0.0f,0.0f,40.0f, 0.0f, 0.0f,6000.0f);
    //Control_Init(0, 0.0f,0.0f, 300.0f, 0.0f,0.0f,40.0f, 0.0f, 0.0f,6000.0f);
    Control_Init(30, 7.5f,40.0f, 300.0f, 5.2f,0.0f,40.0f, 40.0f, 4.5f,6000.0f);
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

    Set_Fan_Speed(Fan_Speed_Target, Fan_Speed_Target);


    static float target_omega = 0.0f;
    static float diff_speed = 0.0f;
    float current_err = camera_error.load(std::memory_order_acquire);

    // 【1. 角度外环】：10ms 执行一次 
    //if(tick % 2 == 0) {
    float err = camera_error.load(std::memory_order_acquire); 
    target_omega = AnglePID_Calc(&Angle_PID, -err); 
    //target_Omega = target_omega;
    //}
    
    // 【2. 角速度内环】：5ms 执行一次 
    float current_gz = mpu_data.gz * 57.29578f; 
    //cur_gz = current_gz; 
    diff_speed = OmegaPID_Calc(&Omega_PID, target_omega, current_gz);
    //dif_speed = diff_speed;

    // 【3. 动力分配与底层速度环】：5ms 执行一次
    float raw_Speed_L, raw_Speed_R;
    Encoder_Speed_Get(&raw_Speed_L, &raw_Speed_R);

    static float filter_L = 0.0f;
    static float filter_R = 0.0f;

    filter_L = 0.8f * raw_Speed_L + 0.2f * filter_L;
    filter_R = 0.8f * raw_Speed_R + 0.2f * filter_R;
    speed_L = filter_L; 
    speed_R = filter_R;


    float dynamic_base_speed = base_speed; 
    error = camera_error.load(std::memory_order_acquire);
    //float xishu = 0.5f; // xishu 越小，外侧车轮加速越少；

    float abs_diff = fabs(diff_speed);
    
    //低速识别模式
    /*if ( fabs(abs_diff) > 5.0f) {
        dynamic_base_speed = base_speed - (abs_diff * 0.6f); 
        if (dynamic_base_speed < 23.0f) {
            dynamic_base_speed =23.0f; 
        }
    }*/    
    //第一跑 保底 30
    if ( fabs(abs_diff) > 5.0f) {
        dynamic_base_speed = base_speed - (abs_diff * 0.5f); 
        if (dynamic_base_speed < 25.0f) {
            dynamic_base_speed =25.0f; 
        }
    }
   //speed change 1 K=0.15 0.1
   //25 40.0 2.1 7.2 15 3.8 1.2

    //中速版本 27
    //40 40.0 2.1 9.4 15 12.8 4.2
    /*if ( fabs(abs_diff) > 5.0f) {
        dynamic_base_speed = base_speed - (abs_diff * 0.5f); 
        if (dynamic_base_speed < 30.0f) {
            dynamic_base_speed =30.0f; 
        }
    }*/
   //speed change 1  k=0.4 0.1

   //中速版本 26
   //电院：43 40.0 2.15 13.2 15 14.6 4.2
   //汽院：43 40.0 2.15 13.2 15 15.8 4.2
   //speed change 1 k=1.2 0.1
    /*if ( fabs(abs_diff) > 5.0f) {
        dynamic_base_speed = base_speed - (abs_diff * 0.5f); 
        if (dynamic_base_speed < 30.0f) {
            dynamic_base_speed =30.0f; 
        }
    }*/
    
    //高速

    /*if ( fabs(abs_diff) > 5.0f) {
        dynamic_base_speed = base_speed - (abs_diff * 0.5f); 
        if (dynamic_base_speed < 33.0f) {
            dynamic_base_speed =33.0f; 
        }
    }*/
    dyna_speed = dynamic_base_speed;

    
    /*int target_L = 0;
    int target_R = 0;*/

    // 判断 diff_speed 的正负来决定转向方向
    // 原逻辑：diff_speed > 0 时，左减右加 -> 左转 (左轮内侧，右轮外侧)
    // 原逻辑：diff_speed < 0 时，左加右减 -> 右转 (右轮内侧，左轮外侧)
   /**if (diff_speed > 0) { 
        // 车辆左转：左轮内侧全额减速，右轮外侧微幅加速
        target_L = dynamic_base_speed - diff_speed/**(1-xishu)*/;
        /*target_R = dynamic_base_speed + (diff_speed * xishu);
    } else if (diff_speed < 0) {
        // 车辆右转：右轮内侧全额减速，左轮外侧微幅加速
        // 注意此时 diff_speed 是负数
        target_L = dynamic_base_speed - (diff_speed * xishu);
        target_R = dynamic_base_speed + diff_speed/**(1-xishu)*/;
    /*} else {
        // 直行
        target_L = dynamic_base_speed;
        target_R = dynamic_base_speed;
    }*/


    int target_L = dynamic_base_speed - diff_speed;
    int target_R = dynamic_base_speed + diff_speed;

    //tar_L = target_L;
    //tar_R = target_R;


    int pwm_left = Left_SpeedPID(&Motor_PID_L, target_L, filter_L);
    int pwm_right = Right_SpeedPID(&Motor_PID_R, target_R, filter_R);
    L_PWM = pwm_left;
    R_PWM = pwm_right;


    if (pwm_left > Motor_PID_L.MaxOut) pwm_left = Motor_PID_L.MaxOut;
    if (pwm_left < -Motor_PID_L.MaxOut) pwm_left = -Motor_PID_L.MaxOut;
    if (pwm_right > Motor_PID_R.MaxOut) pwm_right = Motor_PID_R.MaxOut;
    if (pwm_right < -Motor_PID_R.MaxOut) pwm_right = -Motor_PID_R.MaxOut;


    Left_Motor_Set_Speed((int) pwm_left);
    Right_Motor_Set_Speed((int) pwm_right);
    

    if (tick >= 10000) tick = 0;
}

void UDP_data(void) {
    //char udp_buf[128];
    cout<<"speed_L: "<<speed_L<<"   speed_R: "<<speed_R<<"  target_L: "<<tar_L<<"  target_R: "<<tar_R<<endl;
    cout<<"target_Omega: "<<target_Omega<<"  cur_gz: "<<cur_gz<<"  diff_speed: "<<dif_speed<<"  error: "<<error<<endl;
    cout<<"L_PWM: "<<L_PWM<<"  R_PWM: "<<R_PWM<<endl;
    //cout<<"error: "<<error<<endl;
    cout<<"dynamic_base_speed: "<<dyna_speed<<endl;
    //cout<<"Get_TargetSpeed(): "<<Get_TargetSpeed()<<endl;
    //snprintf(udp_buf, sizeof(udp_buf), "%f,%f,%f\n", error, conf_MAXL*10.0, conf_MAXR*10.0);
    //snprintf(udp_buf, sizeof(udp_buf), "%f\n", (float) error);
    //syd_udp.udp_send_string(udp_buf);
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