/* ============================================================
   store.cpp —— NVS 持久化实现（Preferences，namespace "config"）
   键名与原版 src/configConstants.h 完全一致：
     calib(16B 有符号) / icm_accel0..2 + icm_gyro0..2(float) /
     ID / buzzerVolume / bootSndState / versionDate / birthmark
   从原版固件迁到 Lite 时校准值、设备名不丢。
   与原版的一处刻意差异：固件版本变化不擦除校准（原版会整板
   复位强制重标），只提示日志——实验室开发更友好。
   ============================================================ */
#include <Arduino.h>
#include <Preferences.h>
#include "board.h"
#include "store.h"

static Preferences cfg;

// 键名（改动会在 OTA 迁移时丢用户数据，慎动）
#define KEY_CALIB "calib"
#define KEY_ID "ID"
#define KEY_VOL "buzzerVolume"
#define KEY_SND "bootSndState"
#define KEY_VER "versionDate"
#define KEY_MARK "birthmark"
static const char *IMU_A_KEYS[3] = {"icm_accel0", "icm_accel1", "icm_accel2"};
static const char *IMU_G_KEYS[3] = {"icm_gyro0", "icm_gyro1", "icm_gyro2"};

#define BIRTHMARK '@'
#define FW_VERSION "Lite-261008"  // 改版本时更新（仅用于升级提示，不触发擦除）

// 设备名生成：机型前缀 + 2 位随机十六进制（原版 genBleID 同款）
static void genName(char *buf, uint8_t cap) {
#if defined(BITTLE_R)
  const char *prefix = "BittleR";
#else
  const char *prefix = "Bittle";
#endif
  snprintf(buf, cap, "%s%X%X", prefix,
           (unsigned)(esp_random() & 0xF), (unsigned)(esp_random() & 0xF));
}

void storeSetup() {
  if (!cfg.begin("config", false)) {  // false = 读写模式
    Serial.println("[store] NVS open failed!");
    return;
  }
  if (storeIsNewBoard()) {
    char name[16];
    genName(name, sizeof(name));
    storeWriteName(name);
    storeWriteVolume(5);
    cfg.putBool(KEY_SND, true);
    int8_t zero[DOF] = {};
    storeWriteCalib(zero);  // 校准清零，等待首次标定
    float z6[6] = {};
    storeWriteImuOffset(z6);
    cfg.putString(KEY_VER, FW_VERSION);
    storeMarkAsUsed();
    Serial.printf("[store] new board: name=%s, defaults written\n", name);
  } else {
    String ver = cfg.getString(KEY_VER, "unknown");
    if (ver != FW_VERSION) {
      Serial.printf("[store] firmware %s -> %s, calibration kept\n", ver.c_str(), FW_VERSION);
      cfg.putString(KEY_VER, FW_VERSION);
    }
    char name[16];
    storeReadName(name, sizeof(name));
    Serial.printf("[store] name=%s, free entries=%u\n", name, (unsigned)cfg.freeEntries());
  }
}

void storeReadCalib(int8_t *calib16) {
  if (cfg.getBytes(KEY_CALIB, calib16, DOF) != DOF)
    memset(calib16, 0, DOF);  // 从未写过：视为全零
}

void storeWriteCalib(const int8_t *calib16) {
  cfg.putBytes(KEY_CALIB, calib16, DOF);
}

void storeReadImuOffset(float *offset6) {
  for (int i = 0; i < 3; i++) offset6[i] = cfg.getFloat(IMU_A_KEYS[i], 0.0f);
  for (int i = 0; i < 3; i++) offset6[3 + i] = cfg.getFloat(IMU_G_KEYS[i], 0.0f);
}

void storeWriteImuOffset(const float *offset6) {
  for (int i = 0; i < 3; i++) cfg.putFloat(IMU_A_KEYS[i], offset6[i]);
  for (int i = 0; i < 3; i++) cfg.putFloat(IMU_G_KEYS[i], offset6[3 + i]);
}

bool storeHasImuOffset() {
  return cfg.isKey(IMU_G_KEYS[0]);  // gyro0 有键即认为整组已标定过
}

void storeReadName(char *buf, uint8_t cap) {
  String s = cfg.getString(KEY_ID, "");
  if (s.length() == 0) {  // 兜底：老固件没写过名字
    genName(buf, cap);
    storeWriteName(buf);
    return;
  }
  strncpy(buf, s.c_str(), cap - 1);
  buf[cap - 1] = '\0';
}

void storeWriteName(const char *name) {
  cfg.putString(KEY_ID, name);
}

uint8_t storeReadVolume() {
  int v = cfg.getChar(KEY_VOL, 5);  // 默认 5/10
  return (uint8_t)(v < 0 ? 0 : (v > 10 ? 10 : v));
}

void storeWriteVolume(uint8_t level_0_10) {
  cfg.putChar(KEY_VOL, level_0_10);
}

bool storeIsNewBoard() {
  return cfg.getChar(KEY_MARK, 0) != BIRTHMARK;
}

void storeMarkAsUsed() {
  cfg.putChar(KEY_MARK, BIRTHMARK);
}
