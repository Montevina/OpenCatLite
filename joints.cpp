/* ============================================================
   joints.cpp —— 逻辑关节层实现
   换算链（对照原版 motion.h calibratedPWM + espServo.h:61-62）：
     逻辑角度 -> 限幅 -> 零位+校准 +(角度*方向) -> 脉宽 -> servoPwm
   零位含义：舵机装配中位（量程一半）+ middleShift 偏置，即
   angle=0 时舵机停在"标定姿势"。校准偏移在零位上加减。
   ============================================================ */
#include <Arduino.h>
#include "board.h"
#include "servoPwm.h"
#include "store.h"
#include "joints.h"

static int8_t sCalib[DOF];     // 校准偏移（NVS 存档的副本）
static float sZeroPos[DOF];    // 未校准零位（装配中位，舵机尺度 0~量程）
static float sZeroCal[DOF];    // 校准后零位 = sZeroPos + calib*dir
static float sAngle[DOF];      // 最近一次命令的逻辑角度

void jointsSetup() {
  storeReadCalib(sCalib);
  for (int j = 0; j < DOF; j++) {
    sZeroPos[j] = SERVO_ANGLE_RANGE / 2.0f + (float)JOINT_MIDDLE_SHIFT[j] * JOINT_ROTATION_DIR[j];
    sZeroCal[j] = sZeroPos[j] + (float)sCalib[j] * JOINT_ROTATION_DIR[j];
    sAngle[j] = 0;
  }
  // 不主动驱动舵机：初始保持松开，第一次 jointsSetAngle 才上电（与原版
  // attach 后不回中的行为一致，避免开机瞬间乱动）
  Serial.print("[joints] calib:");
  for (int j = 0; j < DOF; j++)
    if (JOINT_SERVO[j] >= 0) Serial.printf(" %d:%d", j, sCalib[j]);
  Serial.println();
}

int8_t jointsCalib(uint8_t joint) {
  return joint < DOF ? sCalib[joint] : 0;
}

void jointsSetCalib(uint8_t joint, int8_t offset) {
  if (joint >= DOF) return;
  sCalib[joint] = offset;
  sZeroCal[joint] = sZeroPos[joint] + (float)offset * JOINT_ROTATION_DIR[joint];
  // 立即按新零位重发当前角度：校准模式下舵机随偏移实时移动（原版 'c' 行为）
  jointsSetAngle(joint, sAngle[joint]);
}

float jointsZeroPosition(uint8_t joint) {
  return joint < DOF ? sZeroCal[joint] : 0;
}

void jointsSetAngle(uint8_t joint, float angle) {
  if (joint >= DOF) return;
  int8_t servo = JOINT_SERVO[joint];
  if (servo < 0) return;  // 本机型无此关节（取代原版散落各处的 "i>3 && i<8" 判断）
  // 限幅（先限幅再记录，与原版 calibratedPWM 顺序一致）
  if (angle < JOINT_ANGLE_LIMIT[joint][0]) angle = JOINT_ANGLE_LIMIT[joint][0];
  if (angle > JOINT_ANGLE_LIMIT[joint][1]) angle = JOINT_ANGLE_LIMIT[joint][1];
  sAngle[joint] = angle;
  // 舵机尺度角度（0 ~ SERVO_ANGLE_RANGE）
  float dutyDeg = sZeroCal[joint] + angle * JOINT_ROTATION_DIR[joint];
  if (dutyDeg < 0) dutyDeg = 0;
  if (dutyDeg > SERVO_ANGLE_RANGE) dutyDeg = SERVO_ANGLE_RANGE;
  // 角度 -> 脉宽（P1L：270° 对 500~2500µs 线性；P1S 同理 290°）
  uint16_t pulse = (uint16_t)(SERVO_PULSE_MIN_US
                              + dutyDeg * (SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US) / (float)SERVO_ANGLE_RANGE
                              + 0.5f);
  servoPwmWrite(servo, pulse);
}

float jointsAngle(uint8_t joint) {
  return joint < DOF ? sAngle[joint] : 0;
}

void jointsAllOff() {
  servoPwmAllOff();
}
