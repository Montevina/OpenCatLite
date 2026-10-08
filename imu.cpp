/* ============================================================
   imu.cpp —— ICM42670 适配层实现
   数据流：core0 任务每 5ms 调 icm42670_lite 读寄存器 -> Madgwick
   融合（lite 库内）-> 异常判定 -> 自旋锁下发布快照。主循环只调
   imuGetSnapshot。
   参数：IMU_PERIOD 5ms、任务栈 2500、异常阈值表见函数内注释。
   陀螺量程 ±2000dps（原版 ±250 会在手快转时削顶少算角度，
   2026-10-06 实测确认后提高；量化噪声 +12% 相对器件本底可忽略）。
   ============================================================ */
#include <Arduino.h>
#include <Wire.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "board.h"
#include "store.h"
#include "imu.h"
#include "icm42670_lite.h"  // 单文件融合驱动（寄存器+Madgwick 融合一体）

// ---- 驱动实例（address_lsb=1 -> I2C 地址 0x69）----
static ICM42670Lite icm(Wire, 1);

static bool sReady = false;            // 芯片在位且初始化完成
static volatile bool sRunning = false; // 采样任务运行标志
static TaskHandle_t sTask = NULL;

// ---- 快照发布区：自旋锁保护，这是唯一的跨核边界 ----
static portMUX_TYPE sMux = portMUX_INITIALIZER_UNLOCKED;
static ImuSnapshot sPub;
static uint32_t sSeq = 0;

// ---- 任务私有数据（只有任务写，不共享）----
static float sYpr[3], sAcc[3], sPrevAcc[3];
static int sThresX = 5000, sThresY = 4000, sThresZ = 12000;  // 默认姿势档
static const float G_FACTOR = 10.0 / 8192;  // 原版 gFactor，阈值换算用
static const uint8_t IMU_PERIOD_MS = 5;

// I2C 在位探测（对应原版 i2cDetect() 扫到 0x69 置 icmQ 那一行）
static bool probeI2c(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

// ---- 异常判定（对照原版 getImuException）----
// 持续性状态：条件在就保持，消失自动回 NONE
static int8_t detectPersistent() {
  if (fabs(sYpr[2]) > 85 && sAcc[2] < -1)  // ICM 分支阈值 -1（原版 mpuQ/其他 之分）
    return IMU_EX_FLIPPED;
#if !defined(BITTLE_R)  // 原版 #ifndef ROBOT_ARM：带臂机型不做拎起/敲击/推动判定
  if (sYpr[1] < -50 || sYpr[1] > 75)
    return IMU_EX_LIFTED;
#endif
  return IMU_EX_NONE;
}

// 瞬时事件：锁存进快照的 events 位，读走才清，主循环不会漏
static uint8_t detectEvents() {
#if defined(BITTLE_R)
  return 0;
#else
  if (fabs(sAcc[2] - sPrevAcc[2]) > sThresZ * G_FACTOR  // z 向冲击
      && fabs(sAcc[2]) > sThresZ * G_FACTOR)
    return IMU_EV_KNOCKED;
  if ((fabs(sAcc[0] - sPrevAcc[0]) > 4000 * G_FACTOR  // x 向冲击
       && fabs(sAcc[0]) > sThresX * G_FACTOR)
      || (fabs(sAcc[1] - sPrevAcc[1]) > 6000 * G_FACTOR  // y 向冲击
          && fabs(sAcc[1]) > sThresY * G_FACTOR))
    return IMU_EV_PUSHED;
  return 0;
#endif
}

static void publish(int8_t persistent, uint8_t events) {
  portENTER_CRITICAL(&sMux);
  for (uint8_t i = 0; i < 3; i++) {
    sPub.ypr[i] = sYpr[i];
    sPub.accelReal[i] = sAcc[i];
  }
  sPub.exception = persistent;
  sPub.events |= events;  // 累积锁存，不清旧值
  sPub.still = icm.zuptStill();
  sPub.temperature = icm.temperatureC();
  sPub.clipGyro = icm.gyroClipCount;
  sPub.clipAccel = icm.accelClipCount;
  sPub.seq = ++sSeq;
  sPub.timestamp = millis();
  portEXIT_CRITICAL(&sMux);
}

// ---- core 0 采样任务（对照原版 taskIMU，去掉栈水位打印）----
static void imuTask(void *param) {
  (void)param;
  unsigned long lastSample = 0;
  while (sRunning) {
    if (millis() - lastSample > IMU_PERIOD_MS) {
      lastSample = millis();
      icm.getImuGyro();  // 读寄存器 + Madgwick 融合 + 施加 ZUPT yaw 校正
      icm.zuptUpdate();  // ZUPT：静止检测 + 会话零偏重估（机会主义，不阻塞）
      for (uint8_t i = 0; i < 3; i++) {
        sAcc[i] = icm.a_real[i] * 10.0;  // 原版 xyzReal = a_real * GRAVITY
        sYpr[i] = icm.ypr[i];
      }
      sYpr[0] = -sYpr[0];  // 偏航取负 = 极坐标约定（右转为负），同原版
      publish(detectPersistent(), detectEvents());
      for (uint8_t i = 0; i < 3; i++) sPrevAcc[i] = sAcc[i];
    } else {
      vTaskDelay(1 / portTICK_PERIOD_MS);
    }
  }
  sTask = NULL;
  vTaskDelete(NULL);
}

static bool startTask() {
  sRunning = true;
  BaseType_t ok = xTaskCreatePinnedToCore(imuTask, "imu", 2500, NULL, 1, &sTask, 0);
  if (ok != pdPASS) {
    sRunning = false;
    return false;
  }
  return true;
}

// Madgwick 预热收敛等待（原版 waitForImuConvergence 的轻量版）：
// 连续 5 次快照变化量都在阈值内，或超时放弃
static void waitConvergence() {
  ImuSnapshot prev;
  if (!imuGetSnapshot(&prev)) return;
  int stable = 0;
  for (int i = 0; i < 300 && stable < 5; i++) {  // 300*10ms = 最多 3 秒
    delay(10);
    ImuSnapshot cur;
    if (!imuGetSnapshot(&cur) || cur.seq == prev.seq) continue;
    float dy = fabs(cur.ypr[0] - prev.ypr[0]) + fabs(cur.ypr[1] - prev.ypr[1])
               + fabs(cur.ypr[2] - prev.ypr[2]);
    float da = fabs(cur.accelReal[0] - prev.accelReal[0]) + fabs(cur.accelReal[1] - prev.accelReal[1])
               + fabs(cur.accelReal[2] - prev.accelReal[2]);
    if (dy < 0.3 && da < 0.2) stable++;
    else stable = 0;
    prev = cur;
  }
}

// ---- 公开 API ----

void imuSetup(bool calibrateNow) {
  Wire.begin();  // SDA 21 / SCL 22；Lite 里 IMU 是唯一 I2C 设备，这里统管
  if (!probeI2c(ICM42670_I2C_ADDR)) {
    Serial.printf("[IMU] no device at 0x%02X, gyro disabled\n", ICM42670_I2C_ADDR);
    sReady = false;
    return;
  }
  Serial.println("[IMU] ICM42670 found");
  icm.begin();
  icm.init(200, 2, 2000);  // ODR 200Hz / 加速度 ±2g / 陀螺 ±2000dps（见文件头注释）
  delay(10);

  if (storeHasImuOffset()) {
    float off[6];  // [ax ay az gx gy gz]
    storeReadImuOffset(off);
    icm.offset_accel[0] = off[0]; icm.offset_accel[1] = off[1]; icm.offset_accel[2] = off[2];
    icm.offset_gyro[0] = off[3]; icm.offset_gyro[1] = off[4]; icm.offset_gyro[2] = off[5];
    Serial.printf("[IMU] stored offsets: %.1f %.1f %.1f | %.1f %.1f %.1f\n",
                  off[0], off[1], off[2], off[3], off[4], off[5]);
  } else {
    Serial.println("[IMU] no stored offsets yet: ZUPT will learn gyro bias; run 'gc' for full calibration");
    if (calibrateNow) imuCalibrate();
  }

  sReady = startTask();
  if (sReady) waitConvergence();
}

bool imuGetSnapshot(ImuSnapshot *out) {
  if (!out || !sReady) return false;
  portENTER_CRITICAL(&sMux);
  *out = sPub;
  sPub.events = 0;  // 瞬时事件"读走即清"（假设单一消费者：robot 层）
  portEXIT_CRITICAL(&sMux);
  return true;
}

void imuCalibrate() {
  // 对照原版 calibrateICM()。任务在跑先停，标完重启
  bool wasRunning = sRunning;
  if (wasRunning) imuStop();
  Serial.println("[IMU] calibrating...");
  icm.getOffset(200);  // 200 个新样本（DRDY 门控），约 1 秒；结果在函数内完成均值

  if (icm.offset_gyro[0] == -32768 || icm.offset_gyro[1] == -32768 || icm.offset_gyro[2] == -32768) {
    Serial.println("[IMU] read error during calibration (-32768), offsets NOT saved");
  } else {
    float off[6] = {icm.offset_accel[0], icm.offset_accel[1], icm.offset_accel[2],
                    icm.offset_gyro[0], icm.offset_gyro[1], icm.offset_gyro[2]};
    storeWriteImuOffset(off);
    Serial.printf("[IMU] offsets saved: %.1f %.1f %.1f %.1f %.1f %.1f\n",
                  off[0], off[1], off[2], off[3], off[4], off[5]);
    icm.zuptReset();  // 新静态零偏接管全部偏移，ZUPT 会话层清零重来
  }
  if (wasRunning) startTask();
}

void imuStop() {
  sRunning = false;
  // 原版 gc 命令的等待逻辑：轮询任务状态直到它自行退出
  for (int i = 0; i < 100 && sTask != NULL; i++) delay(10);
}

void imuSetKnockThresholds(int x, int y, int z) {
  sThresX = x;
  sThresY = y;
  sThresZ = z;
}
