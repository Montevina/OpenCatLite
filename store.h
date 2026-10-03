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

// IMU 标定偏移（键 "imuOff"，ICM42670 为 6 个 float）
void storeReadImuOffset(float *offset6);
void storeWriteImuOffset(const float *offset6);

// 杂项（键名同原版："ID"/"buzzerVolume"/"bootSndState"/"versionDate"/"birthmark"）
void storeReadName(char *buf, uint8_t cap);
void storeWriteName(const char *name);
uint8_t storeReadVolume();
void storeWriteVolume(uint8_t level_0_10);
bool storeIsNewBoard();   // birthmark != '@'
void storeMarkAsUsed();   // 写 birthmark

#endif
