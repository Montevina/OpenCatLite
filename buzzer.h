/* ============================================================
   buzzer —— L2 驱动层：无源蜂鸣器（非阻塞版）
   与原版 src/sound.h 的区别：beep 只把音符入队立即返回，
   实际发声由 buzzerUpdate() 逐拍推进，不再卡住主循环。
   乐谱格式沿用原版：前半段音高序号、后半段时值代号(1000/d ms)。
   ============================================================ */
#ifndef BUZZER_H
#define BUZZER_H
#include <stdint.h>

void buzzerSetup();
void buzzerUpdate(uint32_t now);            // 主循环 tick：推进当前音符
void buzzerSetVolume(uint8_t level_0_10);
void buzzerBeep(int note, uint16_t durationMs, uint16_t pauseMs = 0, uint8_t repeat = 1);
void buzzerPlayMelody(const uint8_t *melody, uint8_t noteCount);
bool buzzerBusy();

// 内置旋律（数据在 .cpp，从 sound.h 原样搬）
void buzzerPlayBoot();
void buzzerPlayLowBattery();
void buzzerPlayOnBattery();

#endif
