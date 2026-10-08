#include <Arduino.h>
#include "board.h"
#include "buzzer.h"

// TODO: 模块内部静态状态：音符环形队列 + 当前音符截止时刻 + 音量
// TODO: 旋律数据从原版 src/sound.h 搬（melodyNormalBoot/melodyLowBattery/melodyOnBattery）

void buzzerSetup() { /* TODO: LEDC attach 到 PIN_BUZZER，默认音量 5。
                        通道规划已定：用 7 号通道（group 0 / timer 3），
                        独立 timer，避开舵机的 0~3 和 8~15（见 servoPwm.cpp 文件头）*/ }
void buzzerUpdate(uint32_t now) { (void)now; /* TODO: 到期则停声/取下一个音符 */ }
void buzzerSetVolume(uint8_t level_0_10) { (void)level_0_10; /* TODO: 钳位 0~10 */ }
void buzzerBeep(int note, uint16_t durationMs, uint16_t pauseMs, uint8_t repeat) {
  (void)note; (void)durationMs; (void)pauseMs; (void)repeat;  // TODO: 入队
}
void buzzerPlayMelody(const uint8_t *melody, uint8_t noteCount) {
  (void)melody; (void)noteCount;  // TODO: 展开成音符序列入队
}
bool buzzerBusy() { return false; /* TODO */ }
void buzzerPlayBoot() { /* TODO */ }
void buzzerPlayLowBattery() { /* TODO */ }
void buzzerPlayOnBattery() { /* TODO */ }
