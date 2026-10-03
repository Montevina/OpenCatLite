/* ============================================================
   battery —— L2 驱动层：电池电压监测
   挖实现：原版 src/reaction.h 的 lowBattery()。
   保留其双阈值滞回设计（防舵机大电流拉低电压误报），
   去掉它内部的旋律播放和技能加载（那些是 L4 的反应，走事件）。
   ============================================================ */
#ifndef BATTERY_H
#define BATTERY_H
#include <stdint.h>

void batterySetup();
void batteryUpdate(uint32_t now);  // 每秒采样；状态变化发布 EV_LOW_BATTERY/EV_POWER_OK
float batteryVoltage();            // 最近一次换算电压
bool batteryIsLow();

#endif
