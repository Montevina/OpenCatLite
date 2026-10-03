#include <Arduino.h>
#include "board.h"
#include "servoPwm.h"
#include "store.h"
#include "joints.h"

// TODO: 静态状态：servoCalib[DOF]、zeroPosition[DOF]、currentAngle[DOF]
// TODO: jointsSetup: storeReadCalib -> 零位=SERVO_ANGLE_RANGE/2
//   + JOINT_MIDDLE_SHIFT*JOINT_ROTATION_DIR + calib*dir（公式同 espServo.h:61-62）
// TODO: jointsSetAngle: 限幅 -> 零位 + angle*dir -> 角度换脉宽(500~2500us)
//   -> 找 JOINT_PIN[joint]，-1 直接返回（取代原版散落各处的 "i>3 && i<8" 判断）

void jointsSetup() { /* TODO */ }
int8_t jointsCalib(uint8_t joint) { (void)joint; return 0; /* TODO */ }
void jointsSetCalib(uint8_t joint, int8_t offset) { (void)joint; (void)offset; /* TODO */ }
float jointsZeroPosition(uint8_t joint) { (void)joint; return 0; /* TODO */ }
void jointsSetAngle(uint8_t joint, float angle) { (void)joint; (void)angle; /* TODO */ }
float jointsAngle(uint8_t joint) { (void)joint; return 0; /* TODO */ }
void jointsAllOff() { /* TODO */ }
