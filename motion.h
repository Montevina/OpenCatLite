/* ============================================================
   motion —— L3 运动层：多关节目标帧插值（非阻塞）
   给一帧 16 个目标角度，按余弦缓动从当前角度平滑过渡过去。
   姿势切换、步态的逐帧推进、命令 i/m 的联动，最终都落到这里。
   挖实现：原版 src/motion.h transform() 的 else 分支（简单插值）。
   原版 if(0){...} 里 170 行步态过渡实验代码和注释掉的旧版
   transform 一律不搬。delay((DOF-offset)/2) 改为时间片推进。
   ============================================================ */
#ifndef MOTION_H
#define MOTION_H
#include <stdint.h>
#include "board.h"  // DOF

void motionSetup();
void motionStart(const float targetAngles[DOF], float speedRatio);  // 启动一次过渡
void motionUpdate(uint32_t now);   // 主循环 tick：按截止时间推进插值
bool motionBusy();
void motionStop();                 // 立即停在当前角度

#endif
