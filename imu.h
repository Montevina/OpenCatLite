/* ============================================================
   imu —— L2 驱动层：ICM42670 姿态传感器（六轴）
   与原版的关键区别：采样跑在 core 0 的独立任务里，但对外只通过
   imuGetSnapshot() 发布一份加了自旋锁的完整快照。
   原版 ypr/imuException 裸奔在两个核之间（无锁），是竞态源头，
   重构版把跨核边界压缩到"一个结构体 + 一把 portMUX"。
   挖实现：src/imu.h 的 ICM42670 分支（icm42670Setup/readIMU/
   calibrateICM/getImuException）+ src/icm42670/petoi_icm42670p.*
   ============================================================ */
#ifndef IMU_H
#define IMU_H
#include <stdint.h>

// IMU 异常码（数值沿用原版 OpenCat.h IMU_EXCEPTION_*，0 = 正常）
enum ImuException : int8_t {
  IMU_EX_NONE = 0,
  IMU_EX_FLIPPED = -1,     // 翻倒
  IMU_EX_LIFTED = -2,      // 被拎起
  IMU_EX_KNOCKED = -3,     // 被敲
  IMU_EX_PUSHED = -4,      // 被推
  IMU_EX_OFFDIRECTION = -5,  // 走偏
  IMU_EX_FREEFALL = -6,    // 自由落体
  IMU_EX_TURNING = -7,     // 转弯到位
};

struct ImuSnapshot {
  float ypr[3];       // 偏航/俯仰/横滚（度）。ypr[0] 符号约定同原版：右转为负
  float accelReal[3]; // 真实加速度（去掉重力，原版 xyzReal）
  int8_t exception;   // ImuException
  bool updated;       // 换新以来是否已被读过（调试用）
};

void imuSetup(bool calibrateNow);          // Wire 初始化 + 采样任务启动
bool imuGetSnapshot(ImuSnapshot *out);     // 线程安全读快照（随时可调）
void imuCalibrate();                       // 静置标定（阻塞几十秒，标完写 store）
void imuStop();                            // 停采样任务（标定/休眠前）

#endif
