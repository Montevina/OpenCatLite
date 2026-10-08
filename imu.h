/* ============================================================
   imu —— L2 驱动层：ICM42670 姿态传感器（已实现）
   供应商库（InvenSense C 库 + Petoi 封装类，含 Madgwick 融合）
   原样放在 icm42670/ 子目录，本文件只是 OpenCatLite 的适配层。

   与原版的关键区别：
   1. 采样跑在 core 0 任务里，对外只发布一份自旋锁保护的快照，
      取代原版 ypr/imuException 裸奔在两个核之间的写法
   2. 原版散落的 I2C 轮询锁（imuLockI2c 等）全部删除：
      Lite 里 IMU 是唯一 I2C 设备，Wire 库自带总线锁
   3. 异常分两类（原版混在一个变量里）：
      - exception：持续性状态（翻倒/被拎起），条件消失自动清除
      - events：瞬时事件（被敲/被推），锁存到被 imuGetSnapshot 读走
      robot 层不必每个循环都轮询，也不会漏掉 5ms 级的敲击脉冲
   4. 转弯到位判定（原版 turningQ 那段）不属于驱动，留给 robot 层
      用 yaw 快照自行实现
   挖实现对照：原版 src/imu.h 的 icm42670Setup/calibrateICM/readIMU/
   getImuException/taskIMU。
   ============================================================ */
#ifndef IMU_H
#define IMU_H
#include <stdint.h>

// 持续性异常（数值沿用原版 IMU_EXCEPTION_*，0 = 正常）
enum ImuException : int8_t {
  IMU_EX_NONE = 0,
  IMU_EX_FLIPPED = -1,  // 翻倒（|roll|>85 且 z 轴加速度反向）
  IMU_EX_LIFTED = -2,   // 被拎起（俯仰出范围）
};

// 瞬时事件位（读走即清）。自由落体原版就注释掉了，预留位
enum ImuEventBits : uint8_t {
  IMU_EV_KNOCKED = 0x01,  // 被敲（z 向冲击）
  IMU_EV_PUSHED = 0x02,   // 被推（x/y 向冲击）
};

struct ImuSnapshot {
  float ypr[3];        // 偏航/俯仰/横滚（度）。偏航已取负 = 极坐标约定（右转为负），
                       // 输出归一化 (-180,180]；ZUPT 会话零偏已扣除
  float accelReal[3];  // 去重力加速度，单位 1g×10（原版 xyzReal 同款，直立时 z≈+10）
  int8_t exception;    // ImuException，持续性
  uint8_t events;      // ImuEventBits 累积，imuGetSnapshot 读走后清零
  bool still;          // ZUPT 判定"静止中"（robot 层可选利用）
  float temperature;   // 芯片温度 °C（暂只记录不补偿，留作温漂数据分析）
  uint32_t clipGyro;   // 陀螺削顶累计（诊断：高速旋转若仍增长说明超 2000dps）
  uint32_t clipAccel;  // 加速度削顶累计（诊断）
  float yawDrift;      // 偏航漂移补偿累计量（度；调试：应保持小值，巨大=异常）
  float gyroBiasZ;     // Z 轴会话零偏 °/s（ZUPT 学习结果，调试）
  uint32_t seq;        // 快照序号，调用方可用来判断是否更新过
  uint32_t timestamp;  // millis() 时刻
};

void imuSetup(bool calibrateNow);      // Wire 初始化 + 探测 0x69 + 启动采样任务
bool imuGetSnapshot(ImuSnapshot *out); // 线程安全；读走 events；false = 无传感器
void imuCalibrate();                   // 静置标定（内部自动停/启任务，结果写 store）
void imuStop();                        // 停采样任务（标定/休眠前）
void imuSetKnockThresholds(int x, int y, int z);
// 原版 loadBySkillName 的两档阈值：姿势(5000/4000/12000) 步态(12000/10000/15000)，
// 由 robot 层切技能时设置；默认姿势档

#endif
