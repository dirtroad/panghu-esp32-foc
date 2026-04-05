// RDK X5 通信模块头文件

#ifndef RDK_COMM_H
#define RDK_COMM_H

#include <Arduino.h>

struct RdkCommand {
  float target_velocity;    // mm/s
  float target_angular;     // deg/s
  float target_height;      // mm
  float target_roll;        // deg
  bool valid;
};

// 初始化
void rdkInit();

// 循环处理
void rdkLoop();

// 发送状态
void sendStatus(float vel, float pitch, float roll, int battery);

// 获取指令
RdkCommand getCommand();

#endif
