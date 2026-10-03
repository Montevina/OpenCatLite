#include <Arduino.h>
#include "servoFeedback.h"

// TODO: 挖原版 src/espServo.h：
//   measureServoPin / pulseIn 测宽（原版 waitTimeForResponse=10ms 超时、
//   feedbackSignal=3500 判决"舵机在发码"）
//   connectedFeedbackServo[] 连通判定（连续 connectedCountDown 次读到才算）
//   movedJoint[] 静止判定（跟随模式的 SMALL_DIFF 思路）
// 注意：松舵机->测宽->重挂会抖一下，重挂后 delay(12)（原版 reAttachAllServos）

bool servoFeedbackRead(uint8_t joint, float *outAngle) {
  (void)joint; (void)outAngle; return false;  // TODO
}
uint8_t servoFeedbackReadAll(float *outAngles) { (void)outAngles; return 0; /* TODO */ }
void servoFeedbackDetachAll() { /* TODO */ }
void servoFeedbackReattachAll() { /* TODO */ }
