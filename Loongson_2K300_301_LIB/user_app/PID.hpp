#ifndef PID_HPP
#define PID_HPP


typedef struct {
    float Kp, Kd;
    float Kp2,Kd2;
    float Error, Last_Error;
    float Output, Last_Output;
    float MaxOut; // 输出限幅
} AnglePID;   //方向外环 角度 pid

typedef struct {
    float Kp, Kd;
    float Error, Last_Error;
    float Output, Last_Output;
    float MaxOut; // 输出限幅
} OmegaPID;   //方向内环  角速度 pid

typedef struct {
    float Kp, Ki;
    float Error, Last_Error;
    float Output, Last_Output;        // 增量式输出（PWM值）
    float MaxOut;            // 输出限幅
} SpeedPID;   // 速度内环 pid


extern SpeedPID Motor_PID_L;
extern SpeedPID Motor_PID_R; 
extern AnglePID Angle_PID;
extern OmegaPID Omega_PID;



void AnglePID_Init(AnglePID *pid, float Kp, float Kd, float maxOut) ;
void OmegaPID_Init(OmegaPID *pid, float Kp,float Kd, float maxOut) ;
void SpeedPID_Init(SpeedPID *pid, float Kp, float Ki, float maxOut) ;

float AnglePID_Calc(AnglePID *pid, float measure);
float OmegaPID_Calc(OmegaPID *pid, float target, float gyro_z);
float Left_SpeedPID(SpeedPID *pid, float target_speed, float now_speed);
float Right_SpeedPID(SpeedPID *pid, float target_speed, float now_speed);

#endif  