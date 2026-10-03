/* ============================================================
   board.h —— L1 板级层：板型、引脚、硬件常量
   全项目唯一允许出现"裸引脚号/裸电压数"的地方。
   数据全部抄自原版 src/OpenCat.h 的 BiBoard_V1_0 + BITTLE 段。
   （PCA9685/BiBoard2 已废弃，不再保留分支）
   ============================================================ */
#ifndef BOARD_H
#define BOARD_H

// ---- 机型（只放开一个；同板同引脚，差异在舵机型号/限位/技能数据）----
#define BITTLE      // Bittle X：P1L 塑料舵机
// #define BITTLE_R  // Bittle R：P1S 金属舵机 + 头部机械臂（原版 ROBOT_ARM 分支）
// #define NYBBLE    // 预留

#if defined(BITTLE_R)
#define MODEL_NAME "Bittle R"
#else
#define MODEL_NAME "Bittle X"
#endif

// ---- 引脚（BiBoard V1_0，RevDE）----
#define PIN_BUZZER 2   // GPIO2 兼启动引脚，开机瞬间勿拉高
#define PIN_VOLTAGE 37  // 电池 ADC
#define PIN_ANALOG1 34
#define PIN_ANALOG2 35
#define PIN_ANALOG3 36
#define PIN_ANALOG4 39
#define PIN_BACKTOUCH 38
#define PIN_LED_PWM 27
#define PIN_VOICE_RX 26  // 语音模块 UART1
#define PIN_VOICE_TX 25
#define PIN_GROVE_RX2 9  // Grove UART2
#define PIN_GROVE_TX2 10

// ---- 电源（2S 7.4V / 6V 双保险，原版 LOW_VOLTAGE 系列）----
#define BATT_LOW_2S 7.0
#define BATT_NONE_2S 6.8
#define BATT_LOW_6V 5.0
#define BATT_NONE_6V 4.8
// V1_0 实测拟合：V = ADC原始值 / 515 + 1.9（原版 reaction.h lowBattery()）
#define BATT_V_PER_ADC 515.0
#define BATT_V_OFFSET 1.9

// ---- 结构常量 ----
#define DOF 16          // 逻辑关节数（含 Bittle 上不存在的 4~7 号）
#define WALKING_DOF 8   // 实际腿关节数
#define PWM_NUM 12      // 实际舵机数
#define SERIAL_BAUD 115200

// ---- 舵机参数（原版 espServo.h）----
#define SERVO_FREQ_HZ 240        // Petoi 用 240Hz 驱动，不是标准 50Hz
#define SERVO_PULSE_MIN_US 500
#define SERVO_PULSE_MAX_US 2500
#if defined(BITTLE_R)
#define SERVO_ANGLE_RANGE 290    // P1S 金属舵机
#else
#define SERVO_ANGLE_RANGE 270    // P1L 塑料舵机
#endif

// ---- I2C / IMU ----
#define ICM42670_I2C_ADDR 0x69   // 原版 i2cDetect(): 0x68=MPU6050, 0x69=ICM42670

#endif  // BOARD_H
