/* ============================================================
   OpenCatLite —— OpenCat ESP32 固件重构骨架
   ------------------------------------------------------------
   设计原则（对照原版 OpenCatEsp32.ino + src/*.h）：
   1. 分层：只能向下调用，同层不互相调用；数据靠"拉"不靠回调
   2. 非阻塞：任何模块不许 delay()，统一由主循环 tick 推进
   3. 零动态内存：setup 之后不再 new/delete，全部静态缓冲
   4. 显式状态：RobotMode 一个枚举管所有模式，取代原版 90 个 Q 标志
   5. 协议兼容：token 命令保持原版语义（Petoi App 可直接连）

   文件 ↔ 原版对照（实现时的挖掘表）：
     board.h          ← src/OpenCat.h 板级段
     servoPwm.*       ← src/espServo.h（去掉 PCA9685，只留 ESP32 LEDC）
     servoFeedback.*  ← src/espServo.h 反馈部分（Bittle R 自动校准/拖动学习）
     buzzer.*         ← src/sound.h（改为非阻塞队列）
     imu.*            ← src/imu.h 的 ICM42670 部分 + src/icm42670/
     battery.*        ← src/reaction.h 的 lowBattery()
     store.*          ← src/configConstants.h（NVS 部分，去掉 I2C EEPROM）
     joints.*         ← src/OpenCat.h 关节表 + motion.h calibratedPWM()
     motion.*         ← src/motion.h transform()（只留简单插值分支）
     skills.*         ← src/skill.h + InstinctBittleESP.h（值拷贝，不用指针别名）
     balance.*        ← src/motion.h adjust() + adaptiveParameterArray
     taskQueue.*      ← src/taskQueue.h（静态环形缓冲，修复泄漏）
     robot.*          ← src/reaction.h reaction()/dealWithExceptions()
     command.*        ← src/moduleManager.h read_serial() + reaction() switch（改表驱动）
   ============================================================ */
#include <Arduino.h>
#include "board.h"

#include "store.h"
#include "servoPwm.h"
#include "joints.h"
#include "buzzer.h"
#include "imu.h"
#include "battery.h"
#include "balance.h"
#include "skills.h"
#include "motion.h"
#include "taskQueue.h"
#include "robot.h"
#include "command.h"

// —— 临时验证开关（command 层就绪后连同 loop() 里的临时代码一起删除）——
// 置 1 = 开机慢扫头偏航舵机（验证舵机驱动用）。
// 默认 0：扫动给机身注入的振动会干扰 ZUPT 静止检测的上机验证。
#define TEMP_SERVO_SWEEP 0

void setup() {
  Serial.begin(SERIAL_BAUD);

  storeSetup();     // NVS：校准值、名字、音量（其他模块都可能要读写）
  servoPwmSetup();  // LEDC 通道：只碰硬件 PWM，不知道"关节"是什么
  jointsSetup();    // 关节：限幅/方向/零位/校准，换算成脉冲交给 servoPwm
  buzzerSetup();
  imuSetup(false);  // ICM42670，开机默认不重新标定
  batterySetup();
  balanceSetup();
  skillsInit();     // 技能数据表
  motionSetup();
  taskQueueSetup();
  robotSetup();     // 进入 MODE_BOOT -> MODE_REST
  commandSetup();   // 命令分发表 + 各输入源
}

void loop() {
  uint32_t now = millis();

  // tick 顺序 = 数据流动方向：感知 -> 决策 -> 动作 -> 反馈
  batteryUpdate(now);   // 采样电压，低电则发布 EV_LOW_BATTERY
  commandPoll(now);     // 收串口/BLE 解析分发 + 弹出到期任务(CMD_SRC_QUEUE)分发
  robotUpdate(now);     // 状态机：处理事件、驱动 motion/gait/balance
  motionUpdate(now);    // 目标帧插值推进（被 robot 启动）
  balanceUpdate(now);   // 读 IMU 快照，算各关节平衡修正量
  buzzerUpdate(now);    // 非阻塞旋律队列

  // —— 临时：IMU 驱动上机验证（command 层的 'gP' 就绪后删除本段）——
  static uint32_t lastImuPrint = 0;
  if (now - lastImuPrint >= 50) {
    lastImuPrint = now;
    ImuSnapshot snap;
    if (imuGetSnapshot(&snap)) {
      Serial.printf("ypr %7.1f %7.1f %7.1f | acc %6.2f %6.2f %6.2f | ex %d ev 0x%02X | st %d T %4.1f | c %lu/%lu | %lu\n",
                    snap.ypr[0], snap.ypr[1], snap.ypr[2],
                    snap.accelReal[0], snap.accelReal[1], snap.accelReal[2],
                    snap.exception, snap.events, snap.still ? 1 : 0, snap.temperature,
                    (unsigned long)snap.clipGyro, (unsigned long)snap.clipAccel,
                    (unsigned long)snap.seq);
    }
  }

  // —— 临时：舵机上机验证（TEMP_SERVO_SWEEP=1 时启用；'c'/'i' 就绪后删除本段）——
  // 只动 0 号关节（头偏航，最安全）：±30° 慢扫，25°/s
#if TEMP_SERVO_SWEEP
  static uint32_t lastServoSweep = 0;
  static float sweepAngle = 0;
  static int8_t sweepDir = 1;
  if (now - lastServoSweep >= 40) {
    lastServoSweep = now;
    sweepAngle += 1.0f * sweepDir;
    if (sweepAngle >= 30) sweepDir = -1;
    if (sweepAngle <= -30) sweepDir = 1;
    jointsSetAngle(0, sweepAngle);
  }
#endif
}
