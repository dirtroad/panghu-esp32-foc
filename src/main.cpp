// -----------------------------------------------------------------------------
// Copyright (c) 2024 Mu Shibo
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
// -----------------------------------------------------------------------------

// 胖虎机器人 ESP32 固件 - 双轮足平衡控制
// 硬件：ESP32-S3 + SNR8503M 电调 + 轮毂电机×2 + IMU
// 功能：FOC 电机控制 + LQR 自平衡 + WiFi 遥控
// 驱动模式：VSP+FG+DIR (SNR8503M)

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <SimpleFOC.h>
#include <ArduinoJson.h>
#include "Servo_STS3032.h"
#include "basic_web.h"
#include <esp_task_wdt.h>

// ==================== WiFi 配置 ====================
const char* WIFI_SSID = "LaiAiba";
const char* WIFI_PASSWORD = "LaiAi888";

// ==================== 引脚定义 ====================
// SNR8503M 电调驱动引脚 (VSP+FG+DIR 模式)
// 电机 1
#define MOTOR0_VSP_PIN    25   // PWM 速度控制
#define MOTOR0_FG_PIN     32   // 编码器反馈
#define MOTOR0_DIR_PIN    26   // 方向控制
#define MOTOR0_EN_PIN     12   // 使能 (可选)

// 电机 2
#define MOTOR1_VSP_PIN    13   // PWM 速度控制
#define MOTOR1_FG_PIN     33   // 编码器反馈
#define MOTOR1_DIR_PIN    14   // 方向控制
#define MOTOR1_EN_PIN     15   // 使能 (可选)

// IMU 引脚 (MPU6050 I2C)
#define IMU_SDA_PIN   21
#define IMU_SCL_PIN   22

// 舵机控制串口
#define SERVO_SERIAL Serial2
#define SERVO_TX_PIN  17
#define SERVO_RX_PIN  18

// ==================== 全局对象 ====================
WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

// FOC 电机对象 (SNR8503M VSP+FG 模式)
BLDCMotor motor0 = BLDCMotor(14);  // 14 对极
BLDCDriver2PWM driver0 = BLDCDriver2PWM(MOTOR0_VSP_PIN, MOTOR0_EN_PIN);

BLDCMotor motor1 = BLDCMotor(14);
BLDCDriver2PWM driver1 = BLDCDriver2PWM(MOTOR1_VSP_PIN, MOTOR1_EN_PIN);

// 编码器 (FG 信号输入)
Encoder encoder0 = Encoder(MOTOR0_FG_PIN, 33, 1, 4);  // GPIO32=FG, 33=未用
Encoder encoder1 = Encoder(MOTOR1_FG_PIN, 36, 1, 4);  // GPIO33=FG, 36=未用

// IMU 状态
float pitch_angle = 0;
float pitch_rate = 0;
float roll_angle = 0;
float roll_rate = 0;

// 平衡控制参数 (LQR)
float Kp_pitch = 15.0;
float Kd_pitch = 2.5;
float Kp_steer = 3.0;
float Kd_steer = 0.5;

// 控制指令
float target_velocity = 0;      // mm/s
float target_angular = 0;       // deg/s
float target_height = 38;       // mm (舵机高度)
float target_roll = 0;          // deg
bool stable_mode = false;
String dir_command = "stop";

// ==================== 中断回调 ====================
void doEncoder0() {
  encoder0.update();
}

void doEncoder1() {
  encoder1.update();
}

// ==================== WebSocket 回调 ====================
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  if (type == WStype_TEXT) {
    // 解析 JSON 控制指令
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, payload, length);
    
    if (error) return;
    
    if (doc.containsKey("linear")) {
      target_velocity = doc["linear"].as<float>();
    }
    if (doc.containsKey("angular")) {
      target_angular = doc["angular"].as<float>();
    }
    if (doc.containsKey("height")) {
      target_height = doc["height"].as<float>();
    }
    if (doc.containsKey("roll")) {
      target_roll = doc["roll"].as<float>();
    }
    if (doc.containsKey("stable")) {
      stable_mode = doc["stable"].as<int>() == 1;
    }
    if (doc.containsKey("dir")) {
      dir_command = doc["dir"].as<String>();
      
      // 方向按钮覆盖速度指令
      if (dir_command == "forward") target_velocity = 200;
      else if (dir_command == "back") target_velocity = -200;
      else if (dir_command == "stop") target_velocity = 0;
      else if (dir_command == "left") target_angular = 50;
      else if (dir_command == "right") target_angular = -50;
    }
  }
}

// ==================== IMU 读取 ====================
void readIMU() {
  // TODO: 实现 MPU6050 读取
  // 使用 DMP 或互补滤波获取 pitch/roll 角度和角速度
  // 这里先使用模拟数据
  pitch_angle = 0.0;
  pitch_rate = 0.0;
}

// ==================== 平衡控制算法 ====================
float calculateMotorVelocity(int motor_id, float pitch, float pitch_dot, float target_vel, float target_ang) {
  // LQR 平衡控制 + 速度控制
  // u = -K * x
  // x = [pitch, pitch_dot, velocity, velocity_error]
  
  float balance_component = Kp_pitch * pitch + Kd_pitch * pitch_dot;
  float velocity_component = 0.05 * (target_vel - (motor_id == 0 ? motor0.shaft_velocity : motor1.shaft_velocity));
  float steering_component = 0;
  
  if (motor_id == 0) {
    steering_component = Kp_steer * target_ang;
  } else {
    steering_component = -Kp_steer * target_ang;
  }
  
  return balance_component + velocity_component + steering_component;
}

// ==================== 舵机控制 ====================
SMS_STS servoController;
u8 servo_ids[4] = {1, 2, 3, 4};
s16 servo_positions[4];
u16 servo_speeds[4] = {50, 50, 50, 50};
u8 servo_accs[4] = {10, 10, 10, 10};

void updateServos() {
  // 根据 target_height 和 target_roll 计算 4 个舵机目标位置
  // 这里使用简单映射，实际需要运动学逆解
  servo_positions[0] = 512 + target_height * 10 + target_roll * 5;
  servo_positions[1] = 512 + target_height * 10 - target_roll * 5;
  servo_positions[2] = 512 + target_height * 10 + target_roll * 5;
  servo_positions[3] = 512 + target_height * 10 - target_roll * 5;
  
  servoController.SyncWritePosEx(servo_ids, 4, servo_positions, servo_speeds, servo_accs);
}

// ==================== 主控制循环 ====================
unsigned long last_control_time = 0;
const unsigned long CONTROL_INTERVAL = 2;  // 2ms (500Hz)

void controlLoop() {
  unsigned long current_time = millis();
  
  if (current_time - last_control_time >= CONTROL_INTERVAL) {
    last_control_time = current_time;
    
    // 1. 读取传感器
    readIMU();
    motor0.loopFOC();
    motor1.loopFOC();
    
    // 2. 平衡控制解算
    if (stable_mode) {
      float vel0 = calculateMotorVelocity(0, pitch_angle, pitch_rate, target_velocity, target_angular);
      float vel1 = calculateMotorVelocity(1, pitch_angle, pitch_rate, target_velocity, target_angular);
      
      motor0.move(vel0);
      motor1.move(vel1);
    } else {
      motor0.move(0);
      motor1.move(0);
    }
    
    // 3. 更新舵机
    updateServos();
  }
}

// ==================== WiFi 和服务器初始化 ====================
void setupWiFi() {
  Serial.println("Connecting to WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("\nWiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void setupServer() {
  // WebSocket 服务器
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  
  // HTTP 服务器
  server.on("/", []() {
    server.send_P(200, "text/html", basic_web);
  });
  
  server.begin();
  Serial.println("HTTP server started");
}

// ==================== 初始化 ====================
void setup() {
  // 禁用看门狗
  esp_task_wdt_deinit();
  
  Serial.begin(115200);
  Serial.println("\n=== 胖虎机器人 ESP32 固件 ===");
  
  // 初始化舵机串口
  Serial.println("[1/6] 初始化舵机串口...");
  SERVO_SERIAL.begin(115200, SERIAL_8N1, SERVO_RX_PIN, SERVO_TX_PIN);
  servoController.pSerial = &SERVO_SERIAL;
  
  // 初始化编码器
  Serial.println("[2/6] 初始化编码器...");
  encoder0.init();
  encoder0.enableInterrupts(doEncoder0);
  encoder1.init();
  encoder1.enableInterrupts(doEncoder1);
  
  // 初始化 FOC 驱动
  Serial.println("[3/6] 初始化 FOC 驱动...");
  driver0.pwm_frequency = 20000;
  driver0.init();
  motor0.linkDriver(&driver0);
  motor0.controller = MotionControlType::velocity;
  motor0.PID_velocity.P = 0.5;
  motor0.PID_velocity.I = 10;
  motor0.PID_velocity.D = 0;
  motor0.PID_velocity.output_ramp = 1000;
  motor0.PID_velocity.limit = 10;
  motor0.sensor = &encoder0;
  Serial.println("[4/6] 初始化电机 0...");
  motor0.init();
  motor0.initFOC();
  
  driver1.pwm_frequency = 20000;
  driver1.init();
  motor1.linkDriver(&driver1);
  motor1.controller = MotionControlType::velocity;
  motor1.PID_velocity.P = 0.5;
  motor1.PID_velocity.I = 10;
  motor1.PID_velocity.D = 0;
  motor1.PID_velocity.output_ramp = 1000;
  motor1.PID_velocity.limit = 10;
  motor1.sensor = &encoder1;
  Serial.println("[5/6] 初始化电机 1...");
  motor1.init();
  motor1.initFOC();
  
  // 初始化 WiFi 和服务器
  Serial.println("[6/6] 初始化 WiFi...");
  setupWiFi();
  setupServer();
  
  Serial.println("\n=== 初始化完成 ===");
}

// ==================== 主循环 ====================
void loop() {
  webSocket.loop();
  server.handleClient();
  controlLoop();
  delay(1);
}
