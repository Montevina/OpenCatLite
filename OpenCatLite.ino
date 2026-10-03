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
}
