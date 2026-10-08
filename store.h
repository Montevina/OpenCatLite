/* ============================================================
   store —— L2 驱动层：NVS 持久化（Preferences）
   挖实现：原版 src/configConstants.h 的 configSetup()/saveCalib()。
   键名与原版保持一致（namespace "config"），保证从原版固件
   OTA 到 Lite 时用户校准数据不丢。
   原版的 I2C EEPROM 分支已废弃，只留 NVS。
   ============================================================ */
#ifndef STORE_H
#define STORE_H
#include <stdint.h>

void storeSetup();                          // 打开命名空间；首次上电写默认值

// 舵机校准（16 字节，键 "calib"）
void storeReadCalib(int8_t *calib16);
void storeWriteCalib(const int8_t *calib16);

// IMU 标定偏移（6 个 float：accel xyz + gyro xyz；
// 键名同原版 icm_accel0..2 / icm_gyro0..2，注意偏移是"原始 LSB 计数"，
// 陀螺量程变了旧存档即失效，需重标）
void storeReadImuOffset(float *offset6);
void storeWriteImuOffset(const float *offset6);
bool storeHasImuOffset();   // NVS 里是否已有存档（区分"从未标定"与"标定值恰好为 0"）

// 杂项（键名同原版："ID"/"buzzerVolume"/"bootSndState"/"versionDate"/"birthmark"）
void storeReadName(char *buf, uint8_t cap);
void storeWriteName(const char *name);
uint8_t storeReadVolume();
void storeWriteVolume(uint8_t level_0_10);
bool storeIsNewBoard();   // birthmark != '@'
void storeMarkAsUsed();   // 写 birthmark

#endif
