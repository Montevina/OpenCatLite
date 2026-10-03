/* ============================================================
   balance —— L3 运动层：IMU 姿态平衡修正
   周期性拉取 IMU 快照，算出每个关节的平衡修正角；
   robot 层在步行/站立时把修正量叠加到目标角度上。
   挖实现：原版 src/motion.h adjust() + adaptiveParameterArray +
   levelTolerance/RollPitchDeviation 的计算（skill.h perform() 内）。
   原版"系数放大 10 倍存 int8_t"的省内存技巧保留并注释清楚。
   ============================================================ */
#ifndef BALANCE_H
#define BALANCE_H
#include <stdint.h>

void balanceSetup();
void balanceUpdate(uint32_t now);        // 读快照 -> 算 RollPitchDeviation -> 各关节修正
void balanceSetEnabled(bool on);         // 'g' 命令 / 快速动作时关
bool balanceEnabled();
float balanceAdjust(uint8_t joint);      // 当前该关节的修正角（度）
void balanceReset();                     // 清零所有修正

#endif
