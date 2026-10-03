/* ============================================================
   robot —— L4 行为层：RobotMode 显式状态机（全机大脑）
   取代原版的模式隐含在 90 个 Q 标志里（gyroBalanceQ/manualHeadQ/
   walkingQ...）+ reaction() 1300 行 switch。
   规则：
   - 任何时刻只有一个 RobotMode，切换必须过 modeEnter()
   - 安全事件（EV_LOW_BATTERY / EV_FALL_OVER）允许从任意模式
     强制迁移，其余事件按当前模式取舍（取代原版零散的 if）
   挖实现：原版 src/reaction.h 的 dealWithExceptions()（事件判定）
   + skill.h perform()（行为播放，改成逐帧推进不再阻塞）。
   ============================================================ */
#ifndef ROBOT_H
#define ROBOT_H
#include <stdint.h>

enum RobotMode : uint8_t {
  MODE_BOOT,       // 上电流程（版本检查、首次标定提示）
  MODE_REST,       // 趴下松舵（原版 rest/fold 后的省电态）
  MODE_CALIBRATE,  // 标定姿势（c 命令 / 翻倒恢复后）
  MODE_POSTURE,    // 静态姿势 + 平衡微调（sit/up/零位站立）
  MODE_GAIT,       // 步态循环（wkF/bkF/wkL...，支持按圈数/时间/角度停）
  MODE_BEHAVIOR,   // 一次性行为（rc/gg/hd...，逐帧推进可被打断）
  MODE_LOW_POWER,  // 低电保护：趴下、松舵、只保留心跳
};

enum RobotEvent : uint8_t {
  EV_NONE = 0,
  EV_FALL_OVER,    // IMU 翻倒（原版 IMU_EXCEPTION_FLIPPED -> "rc"）
  EV_LIFTED,       // 被拎起（原版 -> "lifted/dropped"）
  EV_KNOCKED,      // 被敲（原版 -> "knock"）
  EV_PUSHED,       // 被推（原版 -> 顺推力方向让步）
  EV_FREEFALL,     // 自由落体（原版 -> "lnd" + 松舵）
  EV_LOW_BATTERY,  // 安全事件，强制 MODE_LOW_POWER
  EV_POWER_OK,     // 电压恢复（battery 层发出）
  EV_SKILL_DONE,   // 步态到圈数 / 行为播完
};

void robotSetup();
void robotUpdate(uint32_t now);        // 状态机主 tick：消化事件 -> 推进当前模式
void robotHandleEvent(RobotEvent ev);  // 各层发布事件的唯一入口
RobotMode robotMode();
const char *robotModeName();           // 调试打印用

// 命令层调用的动作入口（内部会做模式迁移），实现对照原版各 case：
void robotSkill(const char *name);     // 'k'：姿势/步态/行为 三选一
void robotRest();                      // 'd'
void robotCalibratePosture();          // 'c'

// ---- Bittle R 专属（原版 ROBOT_ARM 分支，X 上不可用）----
#if defined(BITTLE_R)
void robotAutoCalibrate();     // 'c 16'：反馈回读 + 参考角全自动校准
void robotCalibratePincer();   // 'c -2'：夹爪堵转振动标定
#endif

#endif
