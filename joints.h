/* ============================================================
   joints —— L3 运动层：逻辑关节模型（已实现）
   一处集中"逻辑角度 -> 物理脉宽"的全部换算：
     限幅 -> 零位(=量程/2 + middleShift*dir) + 校准 -> 角度*方向 -> 脉宽
   换算公式对照原版 src/motion.h calibratedPWM() +
   src/espServo.h attachAllESPServos() 的零位公式（61-62 行）。
   数据表抄自原版 OpenCat.h BITTLE 段；关节->舵机的间接层
   （JOINT_SERVO，等效原版 (i>3)?i-4:i）保留，GPIO 表在 board.h。
   校准值存 NVS（store），改值不自动保存——'s' 命令存（命令层负责）。
   ============================================================ */
#ifndef JOINTS_H
#define JOINTS_H
#include <stdint.h>
#include "board.h"  // DOF

// 关节编号：0=头偏航 1=头俯仰(Bittle X)/臂肩(Bittle R) 2=尾关节(Bittle X)/夹爪(Bittle R) 3=预留
//           4~7 不存在(-1)  8~11=肩  12~15=膝
// 关节 -> 物理舵机号（-1 = 本机型无此关节）；舵机号 -> GPIO 见 board.h SERVO_PIN[]
const int8_t JOINT_SERVO[DOF] = {
    0, 1, 2, 3,          // 头/尾组
    -1, -1, -1, -1,      // Bittle 无此排关节
    4, 5, 6, 7,          // 肩
    8, 9, 10, 11         // 膝
};
// 装配中位偏移（原版 middleShift）
const int8_t JOINT_MIDDLE_SHIFT[DOF] = {
    0, -90, 0, 0,
    -45, -45, -45, -45,
    55, 55, -55, -55,
    -55, -55, -55, -55
};
// 舵机安装方向（原版 rotationDirection）
const int8_t JOINT_ROTATION_DIR[DOF] = {
    1, -1, -1, 1,
    1, -1, 1, -1,
    1, -1, -1, 1,
    -1, 1, 1, -1
};
// 角度限位（原版 angleLimit 用 int，这里必须跟随：步行关节 ±200 超出 int8_t）
const int16_t JOINT_ANGLE_LIMIT[DOF][2] = {
    {-120, 120},
#if defined(BITTLE_R)
    {-10, 180},  // Bittle R 臂肩：可以抬到身后
#else
    {-85, 85},   // Bittle X 头俯仰
#endif
    {-120, 120}, {-120, 120},
    {-90, 60}, {-90, 60}, {-90, 90}, {-90, 90},
    {-200, 80}, {-200, 80}, {-80, 200}, {-80, 200},
    {-80, 200}, {-80, 200}, {-80, 200}, {-80, 200}
};

// 全自动校准参考角（'c 16'，原版 motion.h calibrationReference 按机型二选一）
#if defined(BITTLE_R)
const int8_t JOINT_CALIB_REFERENCE[DOF] = {
    0, 55, 0, 0,
    0, 0, 0, 0,
    65, 65, 77, 77,
    -63, -63, -63, -63
};
#else
const int8_t JOINT_CALIB_REFERENCE[DOF] = {
    0, 0, 0, 0,
    0, 0, 0, 0,
    73, 73, 76, 76,
    -66, -66, -66, -66
};
#endif

void jointsSetup();                                  // 读校准、算零位、回中
int8_t jointsCalib(uint8_t joint);                   // 当前校准值
void jointsSetCalib(uint8_t joint, int8_t offset);   // 校准（c 命令）
float jointsZeroPosition(uint8_t joint);             // 零位脉宽角度
void jointsSetAngle(uint8_t joint, float angle);     // 逻辑角度 -> servoPwm
float jointsAngle(uint8_t joint);                    // 最近一次写入的逻辑角度
void jointsAllOff();                                 // 松开全部舵机

#endif
