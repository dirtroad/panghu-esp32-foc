// 胖虎机器人配置头文件
#ifndef _PANGHU_CONFIG_H
#define _PANGHU_CONFIG_H

// ==================== WiFi 配置 ====================
#define WIFI_SSID           "YOUR_WIFI_SSID"
#define WIFI_PASSWORD       "YOUR_WIFI_PASSWORD"

// ==================== 引脚定义 ====================
// FOC 驱动 (L6234)
#define MOTOR0_U_PIN        25
#define MOTOR0_V_PIN        26
#define MOTOR0_W_PIN        27
#define MOTOR0_EN_PIN       12

#define MOTOR1_U_PIN        13
#define MOTOR1_V_PIN        14
#define MOTOR1_W_PIN        15
#define MOTOR1_EN_PIN       16

// 编码器
#define ENCODER0_A_PIN      32
#define ENCODER0_B_PIN      33
#define ENCODER1_A_PIN      39
#define ENCODER1_B_PIN      36

// IMU (MPU6050)
#define IMU_SDA_PIN         21
#define IMU_SCL_PIN         22

// 舵机串口
#define SERVO_TX_PIN        17
#define SERVO_RX_PIN        18

// ==================== 电机参数 ====================
#define MOTOR_POLE_PAIRS    14          // 电机极对数
#define ENCODER_PPR         1000        // 编码器线数
#define PWM_FREQUENCY       20000       // PWM 频率 (Hz)

// ==================== 控制参数 ====================
#define CONTROL_FREQUENCY   500         // 控制频率 (Hz)
#define CONTROL_INTERVAL    2           // 控制间隔 (ms)

// LQR 平衡参数
#define Kp_PITCH            15.0f       // 俯仰角比例增益
#define Kd_PITCH            2.5f        // 俯仰角微分增益
#define Kp_STEER            3.0f        // 转向比例增益
#define Kd_STEER            0.5f        // 转向微分增益

// 电机速度 PID
#define VEL_PID_P           0.5f
#define VEL_PID_I           10.0f
#define VEL_PID_D           0.0f
#define VEL_PID_RAMP        1000.0f
#define VEL_PID_LIMIT       10.0f

// ==================== 舵机参数 ====================
#define SERVO_COUNT         4
#define SERVO_BAUD          115200
#define SERVO_MIN_POS       0
#define SERVO_MAX_POS       1023
#define SERVO_CENTER_POS    512

// ==================== 安全限制 ====================
#define MAX_PITCH_ANGLE     30.0f       // 最大俯仰角 (度)
#define MAX_ROLL_ANGLE      30.0f       // 最大翻滚角 (度)
#define MAX_LINEAR_VEL      500.0f      // 最大线速度 (mm/s)
#define MAX_ANGULAR_VEL     200.0f      // 最大角速度 (度/s)
#define MIN_BATTERY_VOLT    9.0f        // 最低电池电压 (V)

#endif // _PANGHU_CONFIG_H
