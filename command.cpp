#include <Arduino.h>
#include "board.h"
#include "command.h"
#include "robot.h"
#include "buzzer.h"
#include "joints.h"
#include "store.h"
#include "taskQueue.h"

// ---- 分发表：一行一个命令，就是协议文档 ----
// handler 语义全部对照原版 reaction() 的同名 case：
static void cmdSkill(const char *p, uint8_t l, CmdSource s)     { (void)p;(void)l;(void)s; /* TODO 'k': robotSkill(p) */ }
static void cmdRest(const char *p, uint8_t l, CmdSource s)      { (void)p;(void)l;(void)s; /* TODO 'd': robotRest；带参数=松单个舵机 */ }
static void cmdCalibrate(const char *p, uint8_t l, CmdSource s) { (void)p;(void)l;(void)s; /* TODO 'c': jointsSetCalib 序列 */ }
static void cmdMoveSim(const char *p, uint8_t l, CmdSource s)   { (void)p;(void)l;(void)s; /* TODO 'i': 同时多关节 -> motionStart */ }
static void cmdMoveSeq(const char *p, uint8_t l, CmdSource s)   { (void)p;(void)l;(void)s; /* TODO 'm': 逐个多关节 */ }
static void cmdBeep(const char *p, uint8_t l, CmdSource s)      { (void)p;(void)l;(void)s; /* TODO 'b': buzzerBeep/音量/静音切换 */ }
static void cmdGyro(const char *p, uint8_t l, CmdSource s)      { (void)p;(void)l;(void)s; /* TODO 'g': balanceSetEnabled/子命令 c=imuCalibrate */ }
static void cmdJoints(const char *p, uint8_t l, CmdSource s)    { (void)p;(void)l;(void)s; /* TODO 'j': 打印关节角 */ }
static void cmdBalanceSlope(const char *p, uint8_t l, CmdSource s) { (void)p;(void)l;(void)s; /* TODO 'l': 平衡斜率 -2..2 */ }
static void cmdPause(const char *p, uint8_t l, CmdSource s)     { (void)p;(void)l;(void)s; /* TODO 'p': 步态暂停/恢复 */ }
static void cmdPower(const char *p, uint8_t l, CmdSource s)     { (void)p;(void)l;(void)s; /* TODO 'P': 打印电压 */ }
static void cmdQueue(const char *p, uint8_t l, CmdSource s)     { (void)p;(void)l;(void)s; /* TODO 'q': 解析成 taskQueuePush 序列 */ }
static void cmdSave(const char *p, uint8_t l, CmdSource s)      { (void)p;(void)l;(void)s; /* TODO 's': storeWriteCalib */ }
static void cmdAbort(const char *p, uint8_t l, CmdSource s)     { (void)p;(void)l;(void)s; /* TODO 'a': 放弃校准，读回 */ }
static void cmdReset(const char *p, uint8_t l, CmdSource s)     { (void)p;(void)l;(void)s; /* TODO '!': storeMarkAsUsed + 重启 */ }
static void cmdHelp(const char *p, uint8_t l, CmdSource s)      { (void)p;(void)l;(void)s; /* TODO '?': 打印 COMMAND_TABLE */ }
static void cmdFeedback(const char *p, uint8_t l, CmdSource s)  { (void)p;(void)l;(void)s; /* TODO 'f'/'F': servoFeedbackRead 打印；
                                                                    'fl' 拖动学习 'fr' 回放；'c 16'/'c -2' 见 robot.cpp（Bittle R） */ }

const CmdEntry COMMAND_TABLE[] = {
    {'k', cmdSkill, "技能: k sit / k wkF / k rc；可带圈数/毫秒/角度参数"},
    {'d', cmdRest, "趴下并松舵；d 3 只松 3 号舵机"},
    {'c', cmdCalibrate, "标定姿势；c 关节 偏移 调零；c 99 全自动校准"},
    {'i', cmdMoveSim, "多关节同时动(ASCII): i0 70 8 -20"},
    {'m', cmdMoveSeq, "多关节依次动(ASCII): m0 70 8 -20"},
    {'I', cmdMoveSim, "同 i（二进制编码）"},
    {'M', cmdMoveSeq, "同 m（二进制编码）"},
    {'b', cmdBeep, "蜂鸣: b 音高 时值；单 b 静音切换"},
    {'g', cmdGyro, "平衡开关；gc 重标定 IMU"},
    {'j', cmdJoints, "打印关节角度"},
    {'l', cmdBalanceSlope, "平衡斜率: l 1 1（范围 -2..2）"},
    {'p', cmdPause, "步态暂停/恢复"},
    {'P', cmdPower, "打印电池电压"},
    {'q', cmdQueue, "任务序列: qk sit:1000~m 8 0:500~"},
    {'s', cmdSave, "保存校准到 NVS"},
    {'f', cmdFeedback, "舵机反馈: f 8 读角度；F 跟随；fl 拖动学习 fr 回放"},
    {'a', cmdAbort, "放弃本次校准修改"},
    {'!', cmdReset, "恢复出厂（重启后重新标定）"},
    {'?', cmdHelp, "打印本表"},
};
const uint8_t COMMAND_TABLE_COUNT = sizeof(COMMAND_TABLE) / sizeof(COMMAND_TABLE[0]);

// TODO: 收包状态机（静态缓冲 64 字节起步即可，技能大数据 'K' 后续再议）：
//   参考原版 read_serial() 的终止符约定 + 超时(SERIAL_TIMEOUT=10ms)丢弃
// TODO: commandReply 扇出：回 Serial（+ 未来的 BLE/Web），对照原版
//   printToAllPorts()——注意它内部对 'k'/'m' 的语音模块特判要不要保留

void commandSetup() { /* TODO */ }
void commandPoll(uint32_t now) {
  (void)now;
  // TODO: 1) 先弹到期任务：taskQueuePopIfDue(now, &t)
  //          -> commandInject(t.token, t.params, t.paramLen, CMD_SRC_QUEUE)
  //      2) 再收串口（BLE 预留）完整一条 -> commandInject(..., CMD_SRC_SERIAL)
}
void commandInject(char token, const char *params, uint8_t len, CmdSource src) {
  (void)token; (void)params; (void)len; (void)src;  // TODO: 查表 -> handler(params, len, src)
}
void commandReply(CmdSource src, const char *text) { (void)src; (void)text; /* TODO */ }
void commandReplyChar(CmdSource src, char c) { (void)src; (void)c; /* TODO */ }
