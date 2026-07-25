#include "main.hpp"
#include "Drive.hpp"
#include <signal.h> 
#include <thread>
#include <chrono>
#include <atomic>
#include <iostream>
#include <string>
#include <fstream>

#define USE_MULTI_THREAD 0
using namespace std;
using namespace std::chrono;

// --- 全局变量与原子变量 ---
std::atomic<float> camera_error{0.0f};
std::atomic<bool> stop_flag{false};

int run = 0;           // 1: 开始跑车
int need_restart = 0;  // 1: 需要重启程序
float base_speed = 0.0f;
// 外部变量声明
extern SpeedPID Motor_PID_L;
extern SpeedPID Motor_PID_R;
extern AnglePID Angle_PID;
extern OmegaPID Omega_PID;

extern int16_t Speed_Goal;
extern int16_t bao; 

extern eular_info eulerAngle;
extern mpu_data_t mpu_data;


extern int start_global;
extern int start_test; // 用于测试的全局变量，记录 start 的值


#define COLOR_BLUE   ((lq_display_color_t)0x001F)
#define COLOR_GREEN  ((lq_display_color_t)0x07E0)
#define COLOR_RED    ((lq_display_color_t)0xF800)
#define COLOR_YELLOW ((lq_display_color_t)0xFFE0)

// --- 参数持久化：保存与加载 ---
void Save_Params() {
    ofstream outFile("params.txt");
    if (outFile.is_open()) {
        outFile << Speed_Goal << " " 
                << Motor_PID_L.Kp << " " << Motor_PID_L.Ki << " "
                << Angle_PID.Kp << " " << Angle_PID.Kd;
        outFile.close();
        printf("\n[SYSTEM] 参数已成功保存到 params.txt\n");
    } else {
        printf("\n[ERROR] 无法打开文件保存参数\n");
    }
}

void Load_Params() {
    ifstream inFile("params.txt");
    if (inFile.is_open()) {
        float mkp,mki, akp, akd,Okp, Okd,maxout_omega,maxout_diffspeed,K_TURN;
        int16_t goal;
        if (inFile >> goal >> mkp >> mki >> akp >> akd >> Okp >> Okd >> maxout_omega >> maxout_diffspeed >> K_TURN) {
            Speed_Goal = goal;
            Motor_PID_L.Kp = mkp;
            Motor_PID_R.Kp = mkp;
            Motor_PID_L.Ki = mki;
            Motor_PID_R.Ki = mki;
            Angle_PID.Kp = akp;
            Angle_PID.Kd = akd;
            Omega_PID.Kp = Okp;
            Omega_PID.Kd = Okd;
            Angle_PID.MaxOut = maxout_omega;
            Omega_PID.MaxOut = maxout_diffspeed;
            K_turn = K_TURN;
            Set_Base_Speed((float)Speed_Goal);
            printf("[SYSTEM] 已从文件加载参数: Speed:%d,MKp:%.5f, MKi:%.5f, akp:%.5f,akd:%.5f, Okp:%.5f, Okd:%.5f,maxomega:%.5f,maxdiffspeed:%.5f,K_turn:%.5f\n", 
                goal, mkp,mki,akp,akd,Okp, Okd,maxout_omega,maxout_diffspeed,K_turn);
        }
        inFile.close();
    } else {
        printf("[SYSTEM] 未找到参数文件，使用默认参数\n");
    }
}

// --- 重启与信号处理 ---
void restart_program(void) {
    printf("正在重启程序...\n");
    Left_Motor_Set_Speed(0);
    Right_Motor_Set_Speed(0);
    usleep(500000); 

    char exe_path[PATH_MAX];
    ssize_t len = readlink("/proc/self/exe", exe_path, sizeof(exe_path) - 1);
    if (len != -1) {
        exe_path[len] = '\0';
        char *const argv[] = {exe_path, NULL};
        execve(exe_path, argv, environ);
    }
    exit(1);
}

void smartcar_exit_cleanup_cb(void) 
{
    printf("\n检测到 Ctrl+C，正在紧急注销硬件资源...\n");

    Left_Motor_Set_Speed(0);
    Right_Motor_Set_Speed(0);

    // gyro_timer.stop(); 
    // control_timer.stop();

    usleep(200000); // 延时 200ms 确保底层驱动完全关闭
    printf("[智能车提示] 资源释放完毕，程序安全退出。\n");
}


// --- 终端调参菜单 ---
void Terminal_Debug_Menu() {
    // 调参安全保护：记录状态并停车
    int prev_run = run;
    run = 0;
    Left_Motor_Set_Speed(0);
    Right_Motor_Set_Speed(0);

    int choice;
    float val;
    bool back = false;

    while (!back) {
        printf("\n====== 终端调参模式 (MobaXterm) ======\n");
        printf("1. 目标速度 (Speed_Goal): %d\n", Speed_Goal);
        printf("2. 速度环 Kp (Motor_Kp):  %.5f\n", Motor_PID_L.Kp);
        printf("3. 速度环 Ki (Motor_Ki):  %.5f\n", Motor_PID_L.Ki);
        printf("4. 角度环 Kp (Angle_Kp):  %.5f\n", Angle_PID.Kp);
        printf("5. 角度环 Kd (Angle_Kd):  %.5f\n", Angle_PID.Kd);
        printf("6. 【保存当前参数到文件】\n");
        printf("7. 【执行软件重启】\n");
        printf("0. 退出并恢复运行\n");
        printf("请输入选项: ");

        if (!(cin >> choice)) {
            cin.clear();
            cin.ignore(100, '\n');
            continue;
        }

        if (choice == 0) {
            back = true;
        } else if (choice == 6) {
            Save_Params();
        } else if (choice == 7) {
            restart_program();
        } else if (choice >= 1 && choice <= 5) {
            printf("请输入新数值: ");
            if (cin >> val) {
                switch (choice) {
                    case 1: Speed_Goal = (int16_t)val; Set_Base_Speed(val); break;
                    case 2: Motor_PID_L.Kp = Motor_PID_R.Kp = val; break;
                    case 3: Motor_PID_L.Ki = Motor_PID_R.Ki = val; break;
                    case 4: Angle_PID.Kp = val; break;
                    case 5: Angle_PID.Kd = val; break;
                }
                printf(">>> 更新成功！\n");
            }
        }
    }
    printf(">>> 退出调试，恢复运行状态...\n");
    run = prev_run; // 恢复之前的运行状态
}

void core_control_callback(void) {
    if (!run) {
        Motor_PID_L.Output = 0;
        Motor_PID_R.Output = 0;
        Left_Motor_Set_Speed(0);
        Right_Motor_Set_Speed(0);
        Set_Fan_Speed(0.0f, 0.0f); 
        return; 
    }
    gyro_timer_callback();
    control_callback();
}


// --- 主函数 ---
int main() {
    // 1. 硬件与信号初始化
    Init(); 
    Image_Init();

    lq_signal_set_exit_cb(smartcar_exit_cleanup_cb);

    lq_timer core_timer;
    core_timer.set_seconds_ms(5, core_control_callback);

    
    printf("\n[READY] 系统就绪。按下 KEY_2 进入调参，按下 KEY_0 发车。\n");
    cv::Mat raw_pic,gray_pic;
    while(ls_system_running.load()) {
        if (KEY_0.gpio_level_get() == GPIO_LOW) {
            usleep(50000);
            if (KEY_0.gpio_level_get() == GPIO_LOW) {
                run = !run;
                Load_Params(); 
                if (!run) {
                    Left_Motor_Set_Speed(0);
                    Right_Motor_Set_Speed(0);
                }
                printf(">>> 状态切换: run = %d\n", run);
                while (KEY_0.gpio_level_get() == GPIO_LOW) usleep(10000);
            }
        }

        if(Img_Copy()){
            if(!bypass_action_flag){
                Img_deal();
            }
            Circle_Loop();
            Cross_Loop();
            target_detect_deal();
            //if(Left_Num==0&&Right_Num==0&&Circle_NULL==circle_type&&Cross_NULL==CrossType&&winner==3){
            //    run=0;
            //}
            //cout<<"Circle_type:  "<<circle_type<<endl;
            //cout<<"CrossType:  "<<CrossType<<endl;   
            //cout<<"LRType:  "<<LRType<<endl;
            camera_error.store(MidLine_Ctrl(), std::memory_order_release);
            //cout<<"camera_error:  "<<camera_error<<endl;
            //view();
            //get_pic();
        }
        else {
            this_thread::sleep_for(chrono::milliseconds(100));
        }
    }
    // 资源回收（实际运行中很少到达这里）
    //targetDetector.StopThread();
    stop_flag.store(true);
    return 0;
}

    
    
