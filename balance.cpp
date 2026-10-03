#include <Arduino.h>
#include "imu.h"
#include "balance.h"

// TODO: adaptiveParameterArray[16][2] 从原版 motion.h 搬（LL_LEG 版）
// TODO: adjust() 的限幅/阻尼(ADJUSTMENT_DAMPER)/左右加权逻辑照搬
// TODO: balanceUpdate 只在快照 updated 时重算（参考原版 imuSkip 节流）

void balanceSetup() { /* TODO */ }
void balanceUpdate(uint32_t now) { (void)now; /* TODO */ }
void balanceSetEnabled(bool on) { (void)on; /* TODO */ }
bool balanceEnabled() { return false; /* TODO */ }
float balanceAdjust(uint8_t joint) { (void)joint; return 0; /* TODO */ }
void balanceReset() { /* TODO */ }
