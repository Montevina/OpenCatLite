/* ============================================================
   motion.cpp —— 多关节目标帧插值（非阻塞，第一切片）
   余弦缓动，同原版 transform() 的简单分支：
     dutyAng(s) = target + (1+cos(π·s/steps))/2 × (起始 - 目标)
   s=0 → 起始位，s=steps → 目标位（原版循环含两端，共 steps+1 次写）。
   节奏：每步 8ms（原版 delay((DOF-offset)/2)），speedRatio = 每步度数。
   delay 改为 tick 时间片推进：动作期间主循环照常跑（IMU/命令不受阻）。
   ============================================================ */
#include <Arduino.h>
#include "board.h"
#include "joints.h"
#include "motion.h"

#define MOTION_STEP_MS 8

static float sTarget[DOF];
static float sDiff[DOF];  // 启动时 (当前角 - 目标角)
static int sSteps = 0;
static int sStep = 0;  // 下一个要写的步（第 0 步在启动时立即写）
static uint32_t sLastStepMs = 0;
static bool sBusy = false;

// 写第 s 步（s ∈ [0, steps]；0 = 当前位，steps = 目标位）
static void writeStep(int s) {
  for (int i = 0; i < DOF; i++) {
    float a = sTarget[i];
    if (sSteps > 0)
      a += (1.0f + cosf(M_PI * (float)s / (float)sSteps)) * 0.5f * sDiff[i];
    jointsSetAngle(i, a);
  }
}

void motionSetup() {
  sBusy = false;
}

void motionStart(const float targetAngles[DOF], float speedRatio) {
  float maxDiff = 0;
  for (int i = 0; i < DOF; i++) {
    sTarget[i] = targetAngles[i];
    sDiff[i] = jointsAngle(i) - targetAngles[i];
    float d = fabsf(sDiff[i]);
    if (d > maxDiff) maxDiff = d;
  }
  sSteps = speedRatio > 0 ? (int)lroundf(maxDiff / speedRatio) : 0;
  sStep = 1;
  writeStep(0);  // 第 0 步 = 当前位置：先写一次（无跳变，同原版）
  sLastStepMs = millis();
  sBusy = sSteps > 0;
}

void motionUpdate(uint32_t now) {
  if (!sBusy) return;
  if (now - sLastStepMs < MOTION_STEP_MS) return;
  sLastStepMs = now;
  writeStep(sStep);
  if (sStep >= sSteps) sBusy = false;
  else sStep++;
}

bool motionBusy() {
  return sBusy;
}

void motionStop() {
  sBusy = false;
}
