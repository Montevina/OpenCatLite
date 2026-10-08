# OpenCatLite

OpenCat ESP32 (Bittle) 固件重构。五层架构，模块逐个实现中，已实现：
**imu**（ICM42670 融合驱动 + ZUPT 零速更新）、**store**（NVS 持久化）、
**servoPwm**（LEDC 12 通道舵机输出）、**joints**（逻辑关节->脉宽换算链）。
其余模块为空函数骨架，每个空函数的 TODO 注释标明参考原版（Petoi OpenCatEsp32）
的哪个文件、哪些函数、搬运时要注意什么。

## 架构

```
L5 命令层   command.*      token 协议（兼容 Petoi App），COMMAND_TABLE 表驱动分发
L4 行为层   robot.*        RobotMode 显式状态机 + RobotEvent 事件
            taskQueue.*    静态环形任务队列
L3 运动层   joints.*       逻辑关节：限幅/方向/零位/校准/脉宽换算
            motion.*       余弦缓动插值（非阻塞）
            skills.*       技能数据表（格式沿用原版 Instinct*.h）
            balance.*      IMU 姿态平衡修正
L2 驱动层   servoPwm.*     ESP32 LEDC 舵机脉冲（PCA9685 已废弃）
            servoFeedback.* 舵机角度回读（自动校准/拖动学习依赖）
            buzzer.*       非阻塞蜂鸣器
            imu.*          ICM42670，core0 任务 + 自旋锁快照
            battery.*      电压滞回监测
            store.*        NVS 持久化（键名与原版一致）
L1 板级     board.h        机型/引脚/常量（全项目唯一允许裸引脚号的地方）
```

## 设计原则

- 分层单向调用：只能向下，同层不互调；数据靠拉取不靠回调
- 非阻塞：任何模块不 `delay()`，统一 `xxxUpdate(now)` 时间片推进
- 零动态内存：setup 之后不再 new/delete，全部静态缓冲
- 协议兼容：token 命令语义与原版一致，Petoi App / 串口助手可直接使用

## 机型变体

- `BITTLE`（默认）：Bittle X，P1L 塑料舵机 270°
- `BITTLE_R`：Bittle R，P1S 金属舵机 290° + 头部机械臂（夹爪标定、臂优化步态、重心补偿）

差异集中在 board.h / joints.h 的条件数据和 skills/robot 的 TODO，编译期宏切换。

## 编译验证

`arduino-cli compile --fqbn esp32:esp32:esp32 --warnings default OpenCatLite`（Arduino core 2.0.12），
BITTLE 与 BITTLE_R 两个变体均已通过（约 23% Flash / 7% RAM）。
**必须带 `--warnings default`**：ESP32 核心默认 `-w` 会吞掉全部警告（曾因此让
int8_t 溢出静默通过多轮编译）。Bittle R 变体加
`--build-property "compiler.cpp.extra_flags=-DBITTLE_R"` 验证。
