#include <Arduino.h>
#include "store.h"

// TODO: Preferences config; config.begin("config", false)
// TODO: 新板初始化流程参考原版 configSetup()：
//   生成名字（Bittle+两位随机十六进制，原版 genBleID）、默认音量 5、
//   版本比对触发重置（原版 resetIfVersionOlderThan）

void storeSetup() { /* TODO */ }
void storeReadCalib(int8_t *calib16) { (void)calib16; /* TODO */ }
void storeWriteCalib(const int8_t *calib16) { (void)calib16; /* TODO */ }
void storeReadImuOffset(float *offset6) { (void)offset6; /* TODO */ }
void storeWriteImuOffset(const float *offset6) { (void)offset6; /* TODO */ }
void storeReadName(char *buf, uint8_t cap) { (void)buf; (void)cap; /* TODO */ }
void storeWriteName(const char *name) { (void)name; /* TODO */ }
uint8_t storeReadVolume() { return 5; /* TODO */ }
void storeWriteVolume(uint8_t level_0_10) { (void)level_0_10; /* TODO */ }
bool storeIsNewBoard() { return false; /* TODO */ }
void storeMarkAsUsed() { /* TODO */ }
