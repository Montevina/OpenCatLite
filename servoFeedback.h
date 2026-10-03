/* ============================================================
   servoFeedback —— L2 驱动层：舵机角度反馈（P1S/P1L 回读）
   原理（原版 espServo.h）：松开舵机 -> 舵机把当前位置编码成
   PWM 脉宽发回信号线 -> 测脉宽还原角度 -> 重新挂载。
   用途：Bittle R 的 'c 16' 全自动校准、'fl' 拖动学动作、
   'F' 手掰关节其他腿跟随。没有反馈的舵机返回 false。
   ============================================================ */
#ifndef SERVO_FEEDBACK_H
#define SERVO_FEEDBACK_H
#include <stdint.h>

bool servoFeedbackRead(uint8_t joint, float *outAngle);   // 单关节角度回读
uint8_t servoFeedbackReadAll(float *outAngles);           // 全关节回读，返回成功关节数
void servoFeedbackDetachAll();                            // 持续松舵（跟随/学习模式）
void servoFeedbackReattachAll();                          // 恢复驱动

#endif
