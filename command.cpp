/* ============================================================
   command.cpp —— L5 命令层（第一切片）
   已实现：k（静态姿势）/ c（校准）/ s（保存）/ a（放弃）/ ?（帮助）
   其余 token 保留在表中，触发时明确回"未实现"（不再静默）。
   协议同原版：小写 token + ASCII 参数（空格分隔，'\n' 或 '\r' 结束）。
   完成回显：动作类命令（k、c 无参）在动作结束后回显 token
   （原版串口同步惯例，上位机据此判断动作完成）。
   ============================================================ */
#include <Arduino.h>
#include "board.h"
#include "command.h"
#include "robot.h"
#include "joints.h"
#include "store.h"
#include "motion.h"

static char sLine[32];
static uint8_t sLineLen = 0;
static char sPendingEcho = 0;  // 等动作完成后回显的 token

// ---- 应答（本切片只有 USB 串口一路来源）----
void commandReply(CmdSource src, const char *text) {
  (void)src;
  Serial.println(text);
}
void commandReplyChar(CmdSource src, char c) {
  (void)src;
  Serial.println(c);
}

// ---- 校准值一览（对应原版 'c' 的可视化）----
static void printCalib() {
  Serial.print("[calib]");
  for (int i = 0; i < DOF; i++)
    if (JOINT_SERVO[i] >= 0) Serial.printf(" %d:%d", i, jointsCalib((uint8_t)i));
  Serial.println();
}

// ---- 'k'：技能（第一切片 = 静态姿势）----
static void cmdSkill(const char *p, uint8_t l, CmdSource s) {
  (void)l;
  if (p[0] == '\0') {
    commandReply(s, "usage: k sit | k calib");
    return;
  }
  if (robotSkill(p)) sPendingEcho = 'k';  // 动作完成后回显 k
}

// ---- 'c'：校准 ----
static void cmdCalibrate(const char *p, uint8_t l, CmdSource s) {
  (void)l;
  if (p[0] == '\0') {  // c → 转向校准姿势 + 打印当前校准值
    if (robotCalibratePosture()) sPendingEcho = 'c';
    printCalib();
    return;
  }
  int idx = atoi(p);
  const char *sep = strchr(p, ' ');
  if (sep == NULL) {  // 只有关节号：打印校准值
    printCalib();
    return;
  }
  int val = atoi(sep + 1);
  if (idx == DOF) {  // c 16：全自动校准（依赖舵机角度回读，未实现）
    commandReply(s, "auto calibrate not available yet (needs servoFeedback)");
    return;
  }
  if (idx < 0 || idx >= DOF) {
    commandReply(s, "joint index out of range");
    return;
  }
  // 1001~ 增量约定（原版）：1001=+1°，1009=+9°，负值同理（-1001=-1°）
  if (val >= 1001) val = jointsCalib((uint8_t)idx) + val - 1000;
  else if (val <= -1001) val = jointsCalib((uint8_t)idx) + val + 1000;
  if (val > 100) val = 100;
  if (val < -100) val = -100;
  jointsSetCalib((uint8_t)idx, (int8_t)val);  // 立即按新零位重发（舵机实时动）
  Serial.printf("[calib] joint %d = %d\n", idx, jointsCalib((uint8_t)idx));
  commandReplyChar(s, 'c');
}

// ---- 's'：保存校准到 NVS ----
static void cmdSave(const char *p, uint8_t l, CmdSource s) {
  (void)p; (void)l;
  int8_t calib[DOF];
  for (int i = 0; i < DOF; i++) calib[i] = jointsCalib((uint8_t)i);
  storeWriteCalib(calib);
  commandReply(s, "save offset");
  commandReplyChar(s, 's');
}

// ---- 'a'：放弃改动，恢复存档 ----
static void cmdAbort(const char *p, uint8_t l, CmdSource s) {
  (void)p; (void)l;
  int8_t calib[DOF];
  storeReadCalib(calib);
  for (int i = 0; i < DOF; i++) jointsSetCalib((uint8_t)i, calib[i]);
  commandReply(s, "aborted");
  commandReplyChar(s, 'a');
}

// ---- '?'：帮助（列全部命令表）----
static void cmdHelp(const char *p, uint8_t l, CmdSource s) {
  (void)p; (void)l; (void)s;
  for (uint8_t i = 0; i < COMMAND_TABLE_COUNT; i++)
    Serial.printf(" %c  %s\n", COMMAND_TABLE[i].token, COMMAND_TABLE[i].desc);
}

// ---- 未实现 token 的统一应答 ----
static void cmdTodo(const char *p, uint8_t l, CmdSource s) {
  (void)p; (void)l; (void)s;
  commandReply(s, "not implemented in this build yet");
}

const CmdEntry COMMAND_TABLE[] = {
    {'k', cmdSkill, "技能: k sit / k calib（第一切片仅静态姿势）"},
    {'d', cmdTodo, "趴下并松舵；d 3 只松 3 号舵机"},
    {'c', cmdCalibrate, "校准: c 去校准姿势; c 关节 偏移 调零(1001=+1°); s 存 a 弃"},
    {'i', cmdTodo, "多关节同时动(ASCII): i0 70 8 -20"},
    {'m', cmdTodo, "多关节依次动(ASCII): m0 70 8 -20"},
    {'I', cmdTodo, "同 i（二进制编码）"},
    {'M', cmdTodo, "同 m（二进制编码）"},
    {'b', cmdTodo, "蜂鸣: b 音高 时值；单 b 静音切换"},
    {'g', cmdTodo, "平衡开关；gc 重标定 IMU"},
    {'j', cmdTodo, "打印关节角度"},
    {'l', cmdTodo, "平衡斜率: l 1 1（范围 -2..2）"},
    {'p', cmdTodo, "步态暂停/恢复"},
    {'P', cmdTodo, "打印电池电压"},
    {'q', cmdTodo, "任务序列: qk sit:1000~m 8 0:500~"},
    {'s', cmdSave, "保存校准到 NVS"},
    {'f', cmdTodo, "舵机反馈: f 8 读角度；F 跟随；fl 拖动学习 fr 回放"},
    {'a', cmdAbort, "放弃本次校准修改"},
    {'!', cmdTodo, "恢复出厂（重启后重新标定）"},
    {'?', cmdHelp, "打印本表"},
};
const uint8_t COMMAND_TABLE_COUNT = sizeof(COMMAND_TABLE) / sizeof(COMMAND_TABLE[0]);

// ---- 分发 ----
void commandInject(char token, const char *params, uint8_t len, CmdSource src) {
  for (uint8_t i = 0; i < COMMAND_TABLE_COUNT; i++) {
    if (COMMAND_TABLE[i].token == token) {
      COMMAND_TABLE[i].handler(params, len, src);
      return;
    }
  }
  commandReply(src, "Undefined token!");
  commandReplyChar(src, token);
}

// ---- 串口收行 ----
static void handleLine(char *line) {
  while (*line == ' ') line++;
  if (*line == '\0') return;
  char token = *line;
  char *params = line + 1;
  while (*params == ' ') params++;
  commandInject(token, params, (uint8_t)strlen(params), CMD_SRC_SERIAL);
}

void commandSetup() {
  Serial.println("[command] tokens: k (sit/calib)  c (calibrate)  s (save)  a (abort)  ? (help)");
}

void commandPoll(uint32_t now) {
  (void)now;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (sLineLen > 0) {
        sLine[sLineLen] = '\0';
        handleLine(sLine);
        sLineLen = 0;
      }
    } else if (sLineLen < sizeof(sLine) - 1) {
      sLine[sLineLen++] = c;
    } else {
      sLineLen = 0;  // 超长行丢弃
    }
  }
  // 动作完成回显（原版串口同步惯例）
  if (sPendingEcho != 0 && !motionBusy()) {
    commandReplyChar(CMD_SRC_SERIAL, sPendingEcho);
    sPendingEcho = 0;
  }
}
