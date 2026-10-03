/* ============================================================
   taskQueue —— L4 行为层：静态任务环形队列
   用途：把"坐下 -> 停 1 秒 -> 站起"这类序列排进队列，
   由主循环按到期时间逐条作为命令注入（q 命令的产物也放这里）。
   挖实现：原版 src/taskQueue.h。三处刻意不同：
   1. 固定容量静态数组，杜绝原版 pop 不 delete 的内存泄漏
   2. 参数定长 24 字节，杜绝原版 strcpy(lastCmd, newCmd) 的溢出
   3. 时间比较用无符号差值，避免 millis() 回绕陷阱
   ============================================================ */
#ifndef TASK_QUEUE_H
#define TASK_QUEUE_H
#include <stdint.h>

#define TASKQ_CAPACITY 16
#define TASK_PARAMS_MAX 24

struct Task {
  char token;                       // 复用 command 层的 token
  char params[TASK_PARAMS_MAX];     // 参数（ASCII 或二进制，同 token 约定）
  uint8_t paramLen;
  uint16_t delayMs;                 // 弹出后到下一条的间隔
};

void taskQueueSetup();
bool taskQueuePush(char token, const char *params, uint8_t len, uint16_t delayMs);
bool taskQueuePopIfDue(uint32_t now, Task *out);  // 到期则弹出
bool taskQueueEmpty();
void taskQueueClear();

#endif
