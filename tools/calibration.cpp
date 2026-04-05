// 胖虎机器人校准程序
// 用于编码器零点和电机相位校准

#include <Arduino.h>
#include <SimpleFOC.h>
#include "config.h"

// FOC 电机对象
BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS);
BLDCDriver3PWM driver = BLDCDriver3PWM(MOTOR0_U_PIN, MOTOR0_V_PIN, MOTOR0_W_PIN, MOTOR0_EN_PIN);
Encoder encoder = Encoder(ENCODER0_A_PIN, ENCODER0_B_PIN, ENCODER_PPR);

void doEncoder() {
  encoder.update();
}

// 校准编码器零点和电机相位
void calibrateMotor() {
  Serial.println("=== 开始校准 ===");
  Serial.println("请确保电机可以自由转动");
  delay(2000);
  
  // 1. 初始化编码器
  Serial.println("初始化编码器...");
  encoder.init();
  encoder.enableInterrupts(doEncoder);
  delay(1000);
  
  // 2. 初始化驱动
  Serial.println("初始化驱动...");
  driver.pwm_frequency = PWM_FREQUENCY;
  driver.init();
  motor.linkDriver(&driver);
  
  // 3. 链接编码器
  motor.sensor = &encoder;
  
  // 4. 校准编码器零点和电机相位
  Serial.println("校准编码器和电机相位...");
  Serial.println("电机将缓慢转动，请勿阻挡！");
  delay(3000);
  
  motor.init();
  
  // 5. FOC 初始化
  Serial.println("FOC 初始化...");
  motor.initFOC();
  
  // 6. 显示校准结果
  Serial.println("\n=== 校准完成 ===");
  Serial.print("编码器零点：");
  Serial.println(motor.sensor->getMechanicalAngle());
  Serial.print("电机相位：");
  Serial.println(motor.sensor_direction);
  Serial.print("零点对齐角度：");
  Serial.println(motor.zero_electric_angle);
  
  // 7. 测试转动
  Serial.println("\n=== 测试转动 ===");
  Serial.println("电机将正反转各一圈");
  delay(3000);
  
  motor.controller = MotionControlType::angle;
  motor.P_angle.P = 5.0;
  
  for (int i = 0; i < 3; i++) {
    Serial.println("正转...");
    motor.move(3.14);  // 180 度
    delay(2000);
    
    Serial.println("反转...");
    motor.move(-3.14);
    delay(2000);
  }
  
  motor.move(0);
  Serial.println("测试完成！");
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  
  Serial.println("\n=== 胖虎电机校准工具 ===");
  Serial.println("按任意键开始校准...");
  
  while (!Serial.available());
  Serial.read();
  
  calibrateMotor();
}

void loop() {
  // 校准完成后进入空闲
  delay(1000);
}
