#include <Arduino.h>
#include "joints.h"
#include "motion.h"

// TODO: 静态状态：起始角度帧、目标角度帧、起始时间、总步数、当前步
// TODO: 插值公式照搬原版 transform()：
//   dutyAng = target + (1 + cos(PI * s / steps)) / 2 * (current - target)
//   每步只走 1 度/speedRatio，用 now 差值决定该走第几步（取代 delay）

void motionSetup() { /* TODO */ }
void motionStart(const float targetAngles[DOF], float speedRatio) {
  (void)targetAngles; (void)speedRatio;  // TODO: 记录起始/目标帧，算总步数
}
void motionUpdate(uint32_t now) { (void)now; /* TODO: 逐步调 jointsSetAngle */ }
bool motionBusy() { return false; /* TODO */ }
void motionStop() { /* TODO */ }
