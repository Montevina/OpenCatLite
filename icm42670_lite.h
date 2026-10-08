/* ============================================================
   icm42670_lite.h —— ICM42670 单文件驱动（寄存器 + 数据融合一体）
   ------------------------------------------------------------
   由三层库融合而成（原 InvenSense C 驱动 4600 行 + Arduino 包装
   633 行 + Petoi 封装 340 行，只保留 Bittle 实际用到的路径）：
   - 寄存器地址/位值/初始化序列：改写自 InvenSense ICM42670 官方驱动
     （ISC License, Copyright (c) 2018-2022 TDK InvenSense），
     逐条对照 src（已删除的供应商库）与公开数据手册核对
   - 数据处理与融合（标定/坐标换算/Madgwick/漂移补偿/均值滤波）：
     Kai Mai，源自 Petoi OpenCatEsp32（MIT License, (c) 2021 Rongzhong Li）。
     2026-10-05 理论审查后的清理：算法本身未动，修复了标定重复计数、
     首帧 deltaT 失控、asin 域越界 NaN 风险，偏航漂移补偿加静止门控（见 .cpp）
   - 相比官方路径的刻意简化（都有明确理由）：
     * 不用 FIFO/中断/APEX/自检：Bittle 只轮询数据寄存器
     * 不做寄存器 bank 切换：用到的寄存器复位后全在默认 bank
       （原 transport 的 sreg 路径同样不做 bank 切换）
     * 跳过官方 init_hardware_from_ui 的 FSYNC/时间戳/FIFO/RCOSC
       配置：只影响我们不用的 FIFO 路径和休眠功耗
     * I2C 事务加了返回值检查和错误计数（官方 i2c_read 恒返回 0）
   ============================================================ */
#ifndef ICM42670_LITE_H
#define ICM42670_LITE_H
#include <Arduino.h>
#include <Wire.h>

// ---- 寄存器表（ICM-42670 数据手册寄存器名 -> I2C 地址）----
#define ICM_ADDR_BASE 0x68             // AD0=1 时 0x69（BiBoard 接法）
#define ICM_REG_SIGNAL_PATH_RESET 0x02
#define ICM_SOFT_RESET_VALUE 0x10      // SOFT_RESET_DEVICE_CONFIG_EN = 1<<4
#define ICM_REG_TEMP_DATA1 0x09        // 数据突发起点：temp(2)+accel(6)+gyro(6)=14字节
#define ICM_REG_ACCEL_DATA_X1 0x0B
#define ICM_REG_GYRO_DATA_X1 0x11
#define ICM_REG_PWR_MGMT0 0x1F
#define ICM_PWR_GYRO_LN 0x0C           // GYRO_MODE_LN = 3<<2
#define ICM_PWR_GYRO_MASK 0x0C
#define ICM_PWR_ACCEL_LN 0x03          // ACCEL_MODE_LN = 3<<0
#define ICM_PWR_ACCEL_MASK 0x03
#define ICM_REG_GYRO_CONFIG0 0x20
#define ICM_REG_ACCEL_CONFIG0 0x21
#define ICM_CFG_FS_MASK 0x60           // FS_SEL = bits 7:6? 不，bits 6:5（0x3<<5）
#define ICM_CFG_ODR_MASK 0x0F          // ODR = bits 3:0
#define ICM_REG_INT_STATUS_DRDY 0x39
#define ICM_DRDY_BIT 0x01              // 数据就绪位
#define ICM_REG_INT_STATUS 0x3A
#define ICM_RESET_DONE 0x10            // 软复位完成位（bit 4）
#define ICM_REG_INTF_CONFIG0 0x35
#define ICM_ENDIAN_BIG_BIT 0x10        // bit4：1=数据大端（复位默认）
#define ICM_REG_INTF_CONFIG1 0x36
#define ICM_I3C_SDR_EN 0x08            // bit3：I2C 模式须清除
#define ICM_I3C_DDR_EN 0x04            // bit2：同上
#define ICM_REG_WHO_AM_I 0x75
#define ICM_WHO_AM_I_VALUE 0x67
#define ICM_REG_BLK_SEL_W 0x79         // 间接访问选择器，归零即可（不用间接空间）
#define ICM_REG_BLK_SEL_R 0x7C
#define ICM_MEAN_FILTER_SIZE 4         // 欧拉角均值滤波窗口（原 Petoi 方案）

// ---- ZUPT（零速更新）参数区：上机后按实测振荡残余微调 ----
#define ZUPT_WIN 80            // 统计滑窗样本数（400ms @ 200Hz ODR）
#define ZUPT_CONFIRM_TICKS 60  // 静止确认计数（300ms；慢进）
#define ZUPT_EST_TICKS 20      // 静止期零偏重估周期（100ms）
#define ZUPT_GYRO_STD_TH 0.2f  // 静止判据：三轴角速度 STD 上限（°/s）
#define ZUPT_ACC_STD_TH 0.05f  // 静止判据：加速度模 STD 上限（g）
#define ZUPT_BLEND 0.4f        // 零偏慢混合速率（防单次误判毒化）
#define ZUPT_SLEW_DPS 2.0f     // yaw 校正量输出限速（°/s，平滑无跳变）

class ICM42670Lite {
 public:
  ICM42670Lite(TwoWire &i2c, bool address_lsb);
  int begin();   // 探测 + I2C 模式配置 + 软复位；0=成功
  int init(uint16_t odr, uint16_t accel_fsr_g, uint16_t gyro_fsr_dps);  // 配置并启动传感器
  void getOffset(int num);  // 静置标定（num 个新样本，DRDY 门控）
  void getImuGyro();        // 每个采样 tick 调用；数据无更新时内部自动跳过
  void zuptUpdate();        // ZUPT：静止检测 + 会话零偏重估 + yaw 校正（每 tick 在 getImuGyro 后调）
  void zuptReset();         // 清空 ZUPT 状态（重新标定后调用）
  bool zuptStill() const { return _zs == ZUPT_STATIONARY; }
  float temperatureC() const { return _tempC; }

  volatile uint32_t i2cErrorCount = 0;  // I2C 事务失败累计（供健康监测）
  volatile uint32_t gyroClipCount = 0;   // 陀螺 ADC 削顶累计（高速旋转排查：±2000dps 档下应基本不动）
  volatile uint32_t accelClipCount = 0;  // 加速度 ADC 削顶累计（强冲击场景参考）
  float a_real[3];          // 去重力加速度，单位 g（imu.cpp 读取）
  float ypr[3];             // 偏航/俯仰/横滚，度（偏航未取负，imu.cpp 处理约定）
  float offset_accel[3];    // 标定偏移（原始 LSB 计数），标定结果存 NVS
  float offset_gyro[3];

 private:
  TwoWire *_i2c;
  uint8_t _addr;
  bool _bigEndian;
  // 融合内部状态
  float yawDrift;               // 偏航漂移积分（仅静止时积累，见 Madgwick 内注释）
  float yprHistory[ICM_MEAN_FILTER_SIZE][3];
  int8_t index;
  bool firstRound;
  float accel_ratio, gyro_ratio;  // LSB -> g / -> dps 换算系数
  float _gyroDps[3];            // 标定后角速度（度/秒），漂移门控与融合共用
  float _gbias[3];              // Madgwick 内部自适应陀螺零偏（rad/s）
  uint32_t lastUpdate;
  float deltaT;
  float q[4];
  int16_t _accelRaw[3], _gyroRaw[3], _prevAccelRaw[3];
  int16_t _tempRaw;
  float _tempC;
  // ---- ZUPT 状态（见 .cpp 的状态机注释图）----
  enum ZuptState : uint8_t { ZUPT_MOVING, ZUPT_PENDING, ZUPT_STATIONARY };
  ZuptState _zs;
  int _zuptCnt;                 // PENDING 计数 / STATIONARY 估计分频
  uint32_t _sampleSeq, _zuptSeenSeq;  // 样本锁定：只摄取 DRDY 确认的新样本
  float _ringG[3][ZUPT_WIN];    // 陀螺残差滑窗（已扣静态+会话零偏）
  float _ringA[ZUPT_WIN];       // 加速度模滑窗
  int _ringHead, _ringFill;
  float _sumG[3], _sumSqG[3], _sumA, _sumSqA;  // 滑窗增量统计（O(1) 均值/方差）
  float _sessionBias[3];        // 会话零偏：ZUPT 维护，applyCalibration 里扣除
  float _movingTime;            // 非静止态累积时长（yaw 校正分摊用，秒）
  float _corrTarget, _corrNow;  // yaw 校正：目标值 / 限速逼近中的当前值（度）
  float _medianScratch[ZUPT_WIN];
  int writeReg(uint8_t reg, uint8_t val);
  int readReg(uint8_t reg, uint8_t *buf, uint8_t len);
  bool readData();  // DRDY 检查 + 14 字节突发读 + 解析；false = 无新数据/失败
  void applyCalibration();
  float windowMedian(const float *ring);  // 滑窗中位数（插入排序，静态草稿区）
  void MadgwickQuaternionUpdate(float ax, float ay, float az, float gyrox, float gyroy,
                                float gyroz, float deltaT);
};
#endif
