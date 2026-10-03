#include <Arduino.h>
#include "taskQueue.h"

// TODO: 静态环形缓冲 Task buf[TASKQ_CAPACITY] + 头尾下标 + 上次弹出时刻
// TODO: 到期判定：(uint32_t)(now - lastPopTime) >= 上条 delayMs

void taskQueueSetup() { /* TODO */ }
bool taskQueuePush(char token, const char *params, uint8_t len, uint16_t delayMs) {
  (void)token; (void)params; (void)len; (void)delayMs; return false;  // TODO
}
bool taskQueuePopIfDue(uint32_t now, Task *out) { (void)now; (void)out; return false; /* TODO */ }
bool taskQueueEmpty() { return true; /* TODO */ }
void taskQueueClear() { /* TODO */ }
