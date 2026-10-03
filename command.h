/* ============================================================
   command —— L5 命令层：传输源 + 解析 + 表驱动分发
   取代原版 moduleManager.h read_serial() + reaction() 的巨型 switch。
   协议完全兼容原版（Petoi App / 串口助手可直接用）：
   - 单字符 token；小写 token 参数为 ASCII（'\n' 结尾），
     大写 token 参数为二进制（'~' 结尾）
   - token 表即协议文档：加命令 = 加一行表项
   挖实现：原版 read_serial()（收包/超时/溢出丢弃）；
   各 token 的语义从 reaction() 的对应 case 搬进 handler。
   ============================================================ */
#ifndef COMMAND_H
#define COMMAND_H
#include <stdint.h>

enum CmdSource : uint8_t { CMD_SRC_SERIAL, CMD_SRC_BLE, CMD_SRC_WEB, CMD_SRC_QUEUE };

typedef void (*CmdHandler)(const char *params, uint8_t len, CmdSource src);

struct CmdEntry {
  char token;
  CmdHandler handler;
  const char *desc;   // 打印 '?' 时的帮助文本（取代原版散落的注释）
};

// 分发表（在 command.cpp 定义）。未列出的 token 回 "Undefined token!"
extern const CmdEntry COMMAND_TABLE[];
extern const uint8_t COMMAND_TABLE_COUNT;

void commandSetup();
void commandPoll(uint32_t now);       // 轮询串口（BLE 预留），收齐一条则分发
void commandInject(char token, const char *params, uint8_t len, CmdSource src);  // 队列注入口
void commandReply(CmdSource src, const char *text);  // 应答按来源扇出（原版 printToAllPorts）
void commandReplyChar(CmdSource src, char c);        // 回显 token 表示完成（App 靠它同步）

#endif
