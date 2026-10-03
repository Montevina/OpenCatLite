#include <Arduino.h>
#include "board.h"
#include "battery.h"

// TODO: 状态：低电标志 + 恢复迟滞（原版 +0.2V 缓冲）+ 警告节流计数
// TODO: 换算 v = analogRead(PIN_VOLTAGE)/BATT_V_PER_ADC + BATT_V_OFFSET
// TODO: 状态翻转时 robotHandleEvent(EV_LOW_BATTERY / EV_POWER_OK)

void batterySetup() { /* TODO */ }
void batteryUpdate(uint32_t now) { (void)now; /* TODO */ }
float batteryVoltage() { return 0; /* TODO */ }
bool batteryIsLow() { return false; /* TODO */ }
