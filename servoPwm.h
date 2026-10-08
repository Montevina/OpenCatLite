/* ============================================================
   servoPwm —— L2 驱动层：ESP32 LEDC 舵机脉冲输出（已实现）
   只认"物理舵机号(0~11) + 脉宽(µs)"，不知道角度/关节/校准。
   通道规划（16 个 LEDC 通道；同组相邻两通道共享 timer，共享则必须同频）：
     物理舵机 0~7  -> LEDC 8~15（group 1）
     物理舵机 8~11 -> LEDC 0~3 （group 0 / timer 0,1）
     预留 LEDC 7（group 0 / timer 3）给蜂鸣器——独立 timer，
     蜂鸣器变频率不影响舵机 PWM
   分辨率 16 位 @240Hz（见 .cpp 文件头算账）。
   ============================================================ */
#ifndef SERVO_PWM_H
#define SERVO_PWM_H

#include <stdint.h>

void servoPwmSetup();                       // 配置 12 个通道（240Hz/16位），初始无脉冲（松开）
void servoPwmWrite(uint8_t servo, uint16_t pulseUs);  // 单舵机脉宽（500~2500µs）
void servoPwmOff(uint8_t servo);            // 停输出（松开舵机/省电/反馈模式）
void servoPwmAllOff();

#endif
