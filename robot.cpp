#include <Arduino.h>
#include "robot.h"
#include "imu.h"
#include "battery.h"
#include "joints.h"
#include "motion.h"
#include "skills.h"
#include "balance.h"
#include "taskQueue.h"

// TODO: 内部状态：RobotMode cur + 事件小队列(8 深度静态环形)
// TODO: 每个模式三个函数：bootEnter/bootUpdate/bootExit、restEnter/...
//   命名统一 {mode}Enter / {mode}Update / {mode}Update / {mode}Exit
// TODO: robotUpdate 结构：
//   1) 拉一次 imuGetSnapshot，异常映射成 RobotEvent 入队
//   2) 逐个取事件，按"安全事件强制迁移，其余查当前模式"处理
//      （判定阈值与反应动作对照原版 dealWithExceptions() 各 case，
//        翻倒->"rc"、敲击->"knock"+up、低电->rest+松舵+旋律）
//   3) 调当前模式的 xxxUpdate：
//      MODE_GAIT    -> skillsReadFrame 逐帧喂 motionStart，叠加
//                      balanceAdjust；圈数/时间/角度到量发 EV_SKILL_DONE
//      MODE_BEHAVIOR-> 帧尾的 延时x50ms/触发轴-触发角 改为时间片等待
//                      （原版 while(1) 轮询 IMU 触发的写法废弃）
//      MODE_POSTURE -> 姿势保持 + balance 微调
// TODO: robotSkill('k')：skillsFind -> skillsLoad -> 按 period 迁移到
//   POSTURE/GAIT/BEHAVIOR 之一；'R' 后缀镜像在 load 时做（参考 mirror()）

void robotSetup() { /* TODO: 开机 beep/旋律 + 进 MODE_REST */ }
void robotUpdate(uint32_t now) { (void)now; /* TODO */ }
void robotHandleEvent(RobotEvent ev) { (void)ev; /* TODO */ }
RobotMode robotMode() { return MODE_BOOT; /* TODO */ }
const char *robotModeName() { return "boot"; /* TODO */ }
void robotSkill(const char *name) { (void)name; /* TODO */ }
void robotRest() { /* TODO */ }
void robotCalibratePosture() { /* TODO */ }

// ---- Bittle R 专属（原版 ROBOT_ARM 分支，X 上不可用）----
#if defined(BITTLE_R)
// 'c 16' 全自动校准：趴平 -> servoFeedbackReadAll ->
// 与 JOINT_CALIB_REFERENCE 逐关节求差 -> 写校准（原版 motion.h autoCalibrate）
void robotAutoCalibrate() { /* TODO */ }
// 'c -2' 夹爪标定：小步扫夹爪，IMU 检测堵转振动找到闭合零点
// （原版 calibratePincerByVibration，粗扫步长 4 度再细扫 1 度）
void robotCalibratePincer() { /* TODO */ }
#endif
