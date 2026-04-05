# 胖虎机器人 ESP32 固件

双轮足自平衡机器人 ESP32 底层控制固件

## 📋 功能特性

- ✅ **FOC 电机控制** - 使用 SimpleFOC 实现轮毂电机精准速度控制
- ✅ **LQR 自平衡** - 基于 IMU 的 pitch 角反馈实现自平衡
- ✅ **WiFi 遥控** - WebSocket + Web 界面，支持手机/电脑浏览器控制
- ✅ **舵机同步控制** - STS3032 舵机控制腿部高度和姿态
- ✅ **速度/转向控制** - 支持线速度和角速度指令

## 🛠️ 硬件需求

| 组件 | 型号 | 数量 |
|------|------|------|
| 主控 | ESP32 DevKit v1 | 1 |
| 电机驱动 | L6234 FOC 驱动 | 2 |
| 电机 | 平衡车轮毂电机 | 2 |
| 编码器 | 磁编码器 (AS5600 等) | 2 |
| IMU | MPU6050 | 1 |
| 舵机 | 飞特 STS3032 | 4 |
| 电源 | 3S 锂电池 (11.1V) | 1 |

## 📦 依赖库

项目使用 PlatformIO，自动安装以下库：

- `Simple FOC` v2.3.0+
- `ArduinoJson` v6.21.3+
- `WebSockets` v2.3.7+
- `MPU6050` v1.4.1+

## 🚀 编译与上传

### 1. 安装 PlatformIO

```bash
# 使用 VS Code 安装 PlatformIO 插件
# 或命令行安装
pip install platformio
```

### 2. 配置 WiFi

编辑 `src/main.cpp`，修改 WiFi 配置：

```cpp
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
```

### 3. 编译

```bash
cd 胖虎-esp32-foc
pio run
```

### 4. 上传

```bash
# 自动检测串口
pio run --target upload

# 或指定串口
pio run --target upload --upload-port /dev/cu.usbserial-XXXX
```

### 5. 串口监视

```bash
pio device monitor
```

## 🎮 使用方法

### 1. 启动机器人

- 上电后 ESP32 自动连接 WiFi
- 串口监视器查看 IP 地址：`192.168.x.x`

### 2. 访问控制界面

- 浏览器打开：`http://192.168.x.x`
- 或使用手机连接同一 WiFi 后访问

### 3. 控制方式

- **摇杆**：控制前后左右移动
- **滑块**：
  - BaseHeight：基础高度 (32-85mm)
  - Roll：翻滚角 (-30°~30°)
  - LinearVel：线速度 (-200~200mm/s)
  - AngularVel：角速度 (-100~100°/s)
- **方向按钮**：Forward/Back/Left/Right/Jump
- **Robot Go! 开关**：启用自平衡模式

## 🔧 参数调优

### LQR 平衡参数

在 `src/main.cpp` 中调整：

```cpp
float Kp_pitch = 15.0;   // 俯仰角比例增益
float Kd_pitch = 2.5;    // 俯仰角微分增益
float Kp_steer = 3.0;    // 转向比例增益
float Kd_steer = 0.5;    // 转向微分增益
```

### 电机 PID 参数

```cpp
motor0.PID_velocity.P = 0.5;
motor0.PID_velocity.I = 10;
motor0.PID_velocity.D = 0;
```

## 📐 引脚定义

### FOC 驱动 (L6234)

| 电机 | U 相 | V 相 | W 相 | EN |
|------|------|------|------|----|
| Motor0 | GPIO25 | GPIO26 | GPIO27 | GPIO12 |
| Motor1 | GPIO13 | GPIO14 | GPIO15 | GPIO16 |

### 编码器

| 电机 | A 相 | B 相 |
|------|------|------|
| Encoder0 | GPIO32 | GPIO33 |
| Encoder1 | GPIO39 | GPIO36 |

### 其他外设

| 设备 | 引脚 |
|------|------|
| IMU SDA | GPIO21 |
| IMU SCL | GPIO22 |
| 舵机 TX | GPIO17 |
| 舵机 RX | GPIO18 |

## ⚠️ 注意事项

1. **电源隔离**：ESP32 使用 5V 供电，L6234 使用电池直供 (11.1V)
2. **编码器校准**：首次使用需要校准编码器零点和电机相位
3. **IMU 校准**：上电时保持机器人静止，进行 IMU 零偏校准
4. **安全开关**：建议添加硬件急停开关

## 📝 待办事项

- [ ] 实现 MPU6050 DMP 读取
- [ ] 添加编码器自动校准程序
- [ ] 实现电池电压监测
- [ ] 添加蓝牙调试接口
- [ ] OTA 固件升级

## 📄 许可证

MIT License (继承自世博同学项目)

## 🙏 致谢

- 世博同学提供的代码框架
- SimpleFOC 开源社区
- 飞特舵机文档支持
