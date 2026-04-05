// RDK X5 通信模块
// UART 串口通信：Serial2 (GPIO17=TX, GPIO18=RX)

#include <Arduino.h>
#include <ArduinoJson.h>

// 通信参数
#define RDK_BAUDRATE 115200
#define RDK_SERIAL Serial2
#define RDK_TX_PIN 17
#define RDK_RX_PIN 18

// 接收缓冲区
#define RX_BUFFER_SIZE 256
char rxBuffer[RX_BUFFER_SIZE];
int rxIndex = 0;

// 控制指令
struct RdkCommand {
  float target_velocity;    // mm/s
  float target_angular;     // deg/s
  float target_height;      // mm
  float target_roll;        // deg
  bool valid;
};

RdkCommand rdkCmd = {0, 0, 38, 0, false};

// 状态反馈
struct RdkStatus {
  unsigned long timestamp;
  float actual_velocity;
  float pitch_angle;
  float roll_angle;
  int battery_percent;
};

// 初始化串口
void rdkInit() {
  RDK_SERIAL.begin(RDK_BAUDRATE, SERIAL_8N1, RDK_RX_PIN, RDK_TX_PIN);
  Serial.println("[RDK] 串口初始化完成");
}

// 解析 JSON 指令
void parseCommand(const String& json) {
  StaticJsonDocument<128> doc;
  DeserializationError error = deserializeJson(doc, json);
  
  if (error) {
    Serial.print("[RDK] JSON 解析失败：");
    Serial.println(error.c_str());
    return;
  }
  
  if (doc.containsKey("v")) {
    rdkCmd.target_velocity = doc["v"].as<float>();
  }
  if (doc.containsKey("w")) {
    rdkCmd.target_angular = doc["w"].as<float>();
  }
  if (doc.containsKey("h")) {
    rdkCmd.target_height = doc["h"].as<float>();
  }
  if (doc.containsKey("r")) {
    rdkCmd.target_roll = doc["r"].as<float>();
  }
  
  rdkCmd.valid = true;
  
  Serial.print("[RDK] 收到指令：v=");
  Serial.print(rdkCmd.target_velocity);
  Serial.print(", w=");
  Serial.print(rdkCmd.target_angular);
  Serial.println();
}

// 处理串口数据
void rdkLoop() {
  while (RDK_SERIAL.available()) {
    char c = RDK_SERIAL.read();
    
    if (c == '\n' || c == '\r') {
      // 收到完整行
      if (rxIndex > 0) {
        rxBuffer[rxIndex] = '\0';
        parseCommand(String(rxBuffer));
        rxIndex = 0;
      }
    } else if (rxIndex < RX_BUFFER_SIZE - 1) {
      rxBuffer[rxIndex++] = c;
    }
  }
}

// 发送状态反馈
void sendStatus(float vel, float pitch, float roll, int battery) {
  StaticJsonDocument<128> doc;
  
  doc["ts"] = millis();
  doc["v"] = vel;
  doc["p"] = pitch;
  doc["r"] = roll;
  doc["b"] = battery;
  
  String json;
  serializeJson(doc, json);
  
  RDK_SERIAL.println(json);
}

// 获取最新指令
RdkCommand getCommand() {
  RdkCommand cmd = rdkCmd;
  rdkCmd.valid = false;  // 清除标志
  return cmd;
}
