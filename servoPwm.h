/* ============================================================
   servoPwm —— L2 驱动层：ESP32 LEDC 舵机脉冲输出
   只认"舵机通道号(0~11) + 脉宽(µs)"，不知道角度/关节/校准。
   挖实现：原版 src/espServo.h attachAllESPServos() +
   src/PetoiESP32Servo/ESP32PWM.cpp（通道分配表见 espServo.h 注释）。
   注意原版 ServoModel 的 240Hz/500-2500µs 参数在 board.h 里。
   ============================================================ */
#ifndef SERVO_PWM_H
#define SERVO_PWM_H

#include <stdint.h>

void servoPwmSetup();                       // 占用 LEDC 通道，全部输出到中位
void servoPwmWrite(uint8_t servo, uint16_t pulseUs);  // 单舵机脉宽
void servoPwmOff(uint8_t servo);            // 停输出（松舵机/省电/反馈模式）
void servoPwmAllOff();

#endif
