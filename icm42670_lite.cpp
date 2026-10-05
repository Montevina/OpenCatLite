/* ============================================================
   icm42670_lite.cpp —— 寄存器序列来源标注见 .h 文件头
   I2C 地址 0x69 / 400kHz；轮询式：DRDY -> 14 字节突发 -> 融合
   ============================================================ */
#include "icm42670_lite.h"

// ---- FSR/ODR 数值 -> 寄存器位值（对照官方 to_param 函数的枚举事实）----
static int8_t accelFsrBits(uint16_t g) {
  switch (g) {
    case 2: return 3;
    case 4: return 2;
    case 8: return 1;
    case 16: return 0;
    default: return -1;
  }
}
static int8_t gyroFsrBits(uint16_t dps) {
  switch (dps) {
    case 250: return 3;
    case 500: return 2;
    case 1000: return 1;
    case 2000: return 0;
    default: return -1;
  }
}
static int8_t odrBits(uint16_t hz) {
  switch (hz) {
    case 12: return 0xC;   // 12.5Hz
    case 25: return 0xB;
    case 50: return 0xA;
    case 100: return 0x9;
    case 200: return 0x8;
    case 400: return 0x7;
    case 800: return 0x6;
    case 1600: return 0x5;
    default: return -1;
  }
}

ICM42670Lite::ICM42670Lite(TwoWire &i2c, bool address_lsb) : _i2c(&i2c) {
  _addr = ICM_ADDR_BASE | (address_lsb ? 1 : 0);
  _bigEndian = true;  // 复位默认，begin() 里按 INTF_CONFIG0 实际值刷新
  accel_ratio = gyro_ratio = 0;
  yawDrift = 0;
  index = 0;
  firstRound = true;
  lastUpdate = 0;
  deltaT = 0;
  q[0] = 1.0f;
  q[1] = q[2] = q[3] = 0;
  for (int h = 0; h < ICM_MEAN_FILTER_SIZE; h++)
    yprHistory[h][0] = yprHistory[h][1] = yprHistory[h][2] = 0;
  for (int i = 0; i < 3; i++) {
    a_real[i] = ypr[i] = 0;
    offset_accel[i] = offset_gyro[i] = 0;
    _gyroDps[i] = _gbias[i] = 0;
    _accelRaw[i] = _gyroRaw[i] = _prevAccelRaw[i] = 0;
  }
}

// ---- 底层 I2C 事务（对照官方 i2c_read/i2c_write，补了返回值检查）----
int ICM42670Lite::writeReg(uint8_t reg, uint8_t val) {
  _i2c->beginTransmission(_addr);
  _i2c->write(reg);
  _i2c->write(val);
  if (_i2c->endTransmission() != 0) {
    i2cErrorCount++;
    return -1;
  }
  return 0;
}

int ICM42670Lite::readReg(uint8_t reg, uint8_t *buf, uint8_t len) {
  _i2c->beginTransmission(_addr);
  _i2c->write(reg);
  if (_i2c->endTransmission(false) != 0) {  // repeated start
    i2cErrorCount++;
    return -1;
  }
  if (_i2c->requestFrom(_addr, len) != len) {
    i2cErrorCount++;
    _i2c->endTransmission(true);
    return -1;
  }
  for (uint8_t i = 0; i < len; i++) buf[i] = _i2c->read();
  _i2c->endTransmission(true);
  return 0;
}

// ---- 初始化：序列逐条对照官方 configure_serial_interface + inv_imu_device_reset ----
int ICM42670Lite::begin() {
  uint8_t v;
  delay(3);  // 供电爬升时间上限 3ms（inv_imu_init 第一步）

  // 1. 间接访问选择器归零（官方 reset/串口配置的第一步；本驱动不用间接空间）
  if (writeReg(ICM_REG_BLK_SEL_R, 0)) return -1;
  if (writeReg(ICM_REG_BLK_SEL_W, 0)) return -1;

  // 2. I2C 模式：关闭 I3C（官方 configure_serial_interface 的 I2C 分支）
  if (readReg(ICM_REG_INTF_CONFIG1, &v, 1)) return -1;
  v &= ~(ICM_I3C_SDR_EN | ICM_I3C_DDR_EN);
  if (writeReg(ICM_REG_INTF_CONFIG1, v)) return -1;

  // 3. 软复位，等 1ms，确认 RESET_DONE（inv_imu_device_reset）
  if (writeReg(ICM_REG_SIGNAL_PATH_RESET, ICM_SOFT_RESET_VALUE)) return -1;
  delay(1);
  if (readReg(ICM_REG_INT_STATUS, &v, 1)) return -1;
  if (v != ICM_RESET_DONE) return -3;

  // 4. WHO_AM_I（官方在复位之后检查）
  if (readReg(ICM_REG_WHO_AM_I, &v, 1)) return -1;
  if (v != ICM_WHO_AM_I_VALUE) return -2;

  // 5. 数据字节序（官方 get_endianness；复位默认大端）
  if (readReg(ICM_REG_INTF_CONFIG0, &v, 1)) return -1;
  _bigEndian = (v & ICM_ENDIAN_BIG_BIT) != 0;
  return 0;
}

// ---- 配置并启动：顺序同 Petoi init()（官方 6 个 set/enable 调用的等效内联）----
int ICM42670Lite::init(uint16_t odr, uint16_t accel_fsr_g, uint16_t gyro_fsr_dps) {
  int8_t afs = accelFsrBits(accel_fsr_g);
  int8_t gfs = gyroFsrBits(gyro_fsr_dps);
  int8_t ob = odrBits(odr);
  if (afs < 0 || gfs < 0 || ob < 0) return -1;
  uint8_t v;

  // ACCEL_CONFIG0 / GYRO_CONFIG0：FS_SEL 与 ODR 位段不重叠，
  // 官方两次读-改-写合并为一次（结果一致）
  if (readReg(ICM_REG_ACCEL_CONFIG0, &v, 1)) return -1;
  v = (v & ~(ICM_CFG_FS_MASK | ICM_CFG_ODR_MASK)) | ((uint8_t)afs << 5) | (uint8_t)ob;
  if (writeReg(ICM_REG_ACCEL_CONFIG0, v)) return -1;

  if (readReg(ICM_REG_GYRO_CONFIG0, &v, 1)) return -1;
  v = (v & ~(ICM_CFG_FS_MASK | ICM_CFG_ODR_MASK)) | ((uint8_t)gfs << 5) | (uint8_t)ob;
  if (writeReg(ICM_REG_GYRO_CONFIG0, v)) return -1;

  // PWR_MGMT0：先加速度计 LN 再陀螺 LN，各等 200µs（官方 enable_*_low_noise_mode）。
  // 冷启动两模式均为 OFF，官方的 RCOSC 切换等待分支不会触发。
  if (readReg(ICM_REG_PWR_MGMT0, &v, 1)) return -1;
  v = (v & ~ICM_PWR_ACCEL_MASK) | ICM_PWR_ACCEL_LN;
  if (writeReg(ICM_REG_PWR_MGMT0, v)) return -1;
  delayMicroseconds(200);
  if (readReg(ICM_REG_PWR_MGMT0, &v, 1)) return -1;
  v = (v & ~ICM_PWR_GYRO_MASK) | ICM_PWR_GYRO_LN;
  if (writeReg(ICM_REG_PWR_MGMT0, v)) return -1;
  delayMicroseconds(200);

  // 换算系数（原 Petoi init() 公式）
  accel_ratio = 32768.0f / accel_fsr_g;
  gyro_ratio = 131.0f / (gyro_fsr_dps / 250.0f);
  yawDrift = 0;
  index = 0;
  firstRound = true;
  for (int h = 0; h < 4; h++)
    yprHistory[h][0] = yprHistory[h][1] = yprHistory[h][2] = 0;
  return 0;
}

// ---- 数据读取：DRDY 检查 + 一次突发替代官方的三次分散读（寄存器地址连续）----
bool ICM42670Lite::readData() {
  uint8_t st;
  if (readReg(ICM_REG_INT_STATUS_DRDY, &st, 1)) return false;
  if (!(st & ICM_DRDY_BIT)) return false;
  uint8_t b[14];  // temp(2) + accel(6) + gyro(6)
  if (readReg(ICM_REG_TEMP_DATA1, b, 14)) return false;
  for (int i = 0; i < 6; i++) {
    uint8_t hi = b[2 + i * 2], lo = b[3 + i * 2];  // 前两字节是温度，跳过
    int16_t val = _bigEndian ? (int16_t)(((uint16_t)hi << 8) | lo)
                             : (int16_t)(((uint16_t)lo << 8) | hi);
    if (i < 3)
      _accelRaw[i] = val;
    else
      _gyroRaw[i - 3] = val;
  }
  return true;
}

// ---- 数据处理：标定 / 坐标换算 / 融合调度 ----
// 标定原理：陀螺零输入时输出恒为零偏 -> 均值即零偏；
//           加速度计水平放置时 Z 轴承 1g -> X/Y 均值即零偏，
//           Z 均值减去整 1g（accel_ratio 个 LSB）即 Z 零偏。
// 注意：此法假设板子严格水平，倾斜时重力会串进 X/Y（上机流程已约定"平放"）。
void ICM42670Lite::getOffset(int num) {
  Serial.printf("[ICM] calibrating: %d samples, keep still and flat...\n", num);
  int32_t acc[3] = { 0 }, gyr[3] = { 0 };  // 整数精确累加，避免 float 丢低位
  int collected = 0;
  while (collected < num) {
    if (readData()) {  // 只统计 DRDY 确认的新样本；旧版重复累加过期值
      for (int i = 0; i < 3; i++) {
        acc[i] += _accelRaw[i];
        gyr[i] += _gyroRaw[i];
      }
      collected++;
    }
    delay(5);  // 200Hz ODR 一个周期
  }
  for (int i = 0; i < 3; i++) {
    offset_accel[i] = (float)acc[i] / num;
    offset_gyro[i] = (float)gyr[i] / num;
  }
  offset_accel[2] -= accel_ratio;  // 去掉水平放置时的 1g
}

// 输出 = (原始计数 - 零偏) / 满量程换算系数；陀螺另存一份 deg/s 供门控用
void ICM42670Lite::applyCalibration() {
  for (int i = 0; i < 3; i++) {
    a_real[i] = (_accelRaw[i] - offset_accel[i]) / accel_ratio;  // g
    _gyroDps[i] = (_gyroRaw[i] - offset_gyro[i]) / gyro_ratio;   // deg/s
  }
}

void ICM42670Lite::getImuGyro() {
  if (!readData()) return;  // 无新数据（旧版靠旧值比较间接实现同样的跳过）
  if (_accelRaw[0] != _prevAccelRaw[0] || _accelRaw[1] != _prevAccelRaw[1]
      || _accelRaw[2] != _prevAccelRaw[2]) {
    for (int i = 0; i < 3; i++) _prevAccelRaw[i] = _accelRaw[i];
    applyCalibration();
    uint32_t now = micros();
    deltaT = ((now - lastUpdate) / 1000000.0f);
    lastUpdate = now;
    if (deltaT <= 0.0f || deltaT > 0.1f)  // 首帧/任务长阻塞保护：按一个采样周期算
      deltaT = 0.005f;
    // 陀螺速率转 rad/s 后进融合
    MadgwickQuaternionUpdate(a_real[0], a_real[1], a_real[2], _gyroDps[0] * DEG_TO_RAD,
                             _gyroDps[1] * DEG_TO_RAD, _gyroDps[2] * DEG_TO_RAD, deltaT);
  }
}

// Sebastian Madgwick 六轴定向滤波（加速度计 + 陀螺，无磁力计）。
// 原理两句话：用陀螺积分做姿态"预测"，再用"加速度计测得的重力方向 vs
// 预测姿态给出的重力方向"之差做梯度下降"校正"；beta 是对加速度计的
// 信任度（越大收敛越快、越抖），zeta 项在线估计陀螺零偏慢漂移。
// 算法与 Petoi/原 Madgwick 实现逐项一致，仅做了三处健壮性修补（见行内注释）。
void ICM42670Lite::MadgwickQuaternionUpdate(float ax, float ay, float az, float gyrox,
                                            float gyroy, float gyroz, float deltaT) {
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];         // 四元数简写
  float norm;                                               // 向量模
  float f1, f2, f3;                                         // 目标函数（重力方向误差）
  float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // 雅可比元素
  float qDot1, qDot2, qDot3, qDot4;
  float hatDot1, hatDot2, hatDot3, hatDot4;
  float gerrx, gerry, gerrz;                                // 陀螺零偏误差估计
  // beta = sqrt(3/4) * 40°/s：收敛快、静止时稍抖（可调小换平滑）
  float GyroMeasError = PI * (40.0f / 180.0f);
  float beta = sqrt(3.0f / 4.0f) * GyroMeasError;
  float GyroMeasDrift = PI * (2.0f / 180.0f);     // 零偏漂移增益
  float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;

  // Auxiliary variables to avoid repeated arithmetic
  float _halfq1 = 0.5f * q1;
  float _halfq2 = 0.5f * q2;
  float _halfq3 = 0.5f * q3;
  float _halfq4 = 0.5f * q4;
  float _2q1 = 2.0f * q1;
  float _2q2 = 2.0f * q2;
  float _2q3 = 2.0f * q3;
  float _2q4 = 2.0f * q4;

  // Normalise accelerometer measurement
  norm = sqrt(ax * ax + ay * ay + az * az);
  if (norm == 0.0f)
    return;  // handle NaN
  norm = 1.0f / norm;
  ax *= norm;
  ay *= norm;
  az *= norm;

  // Compute the objective function and Jacobian
  f1 = _2q2 * q4 - _2q1 * q3 - ax;
  f2 = _2q1 * q2 + _2q3 * q4 - ay;
  f3 = 1.0f - _2q2 * q2 - _2q3 * q3 - az;
  J_11or24 = _2q3;
  J_12or23 = _2q4;
  J_13or22 = _2q1;
  J_14or21 = _2q2;
  J_32 = 2.0f * J_14or21;
  J_33 = 2.0f * J_11or24;

  // Compute the gradient (matrix multiplication)
  hatDot1 = J_14or21 * f2 - J_11or24 * f1;
  hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
  hatDot3 = J_12or23 * f2 - J_33 * f3 - J_13or22 * f1;
  hatDot4 = J_14or21 * f1 + J_11or24 * f2;

  // Normalize the gradient
  norm = sqrt(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
  hatDot1 /= norm;
  hatDot2 /= norm;
  hatDot3 /= norm;
  hatDot4 /= norm;

  // Compute estimated gyroscope biases
  gerrx = _2q1 * hatDot2 - _2q2 * hatDot1 - _2q3 * hatDot4 + _2q4 * hatDot3;
  gerry = _2q1 * hatDot3 + _2q2 * hatDot4 - _2q3 * hatDot1 - _2q4 * hatDot2;
  gerrz = _2q1 * hatDot4 - _2q2 * hatDot3 + _2q3 * hatDot2 - _2q4 * hatDot1;

  // Compute and remove gyroscope biases
  _gbias[0] += gerrx * deltaT * zeta;
  _gbias[1] += gerry * deltaT * zeta;
  _gbias[2] += gerrz * deltaT * zeta;
  gyrox -= _gbias[0];
  gyroy -= _gbias[1];
  gyroz -= _gbias[2];

  // Compute the quaternion derivative
  qDot1 = -_halfq2 * gyrox - _halfq3 * gyroy - _halfq4 * gyroz;
  qDot2 = _halfq1 * gyrox + _halfq3 * gyroz - _halfq4 * gyroy;
  qDot3 = _halfq1 * gyroy - _halfq2 * gyroz + _halfq4 * gyrox;
  qDot4 = _halfq1 * gyroz + _halfq2 * gyroy - _halfq3 * gyrox;

  // Compute then integrate estimated quaternion derivative
  q1 += (qDot1 - (beta * hatDot1)) * deltaT;
  q2 += (qDot2 - (beta * hatDot2)) * deltaT;
  q3 += (qDot3 - (beta * hatDot3)) * deltaT;
  q4 += (qDot4 - (beta * hatDot4)) * deltaT;

  // Normalize the quaternion
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);  // normalise quaternion
  norm = 1.0f / norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;

  // ---- 四元数 -> 欧拉角（含 4 点均值滤波与偏航漂移补偿，原 Petoi 方案）----
  yprHistory[index][0] = -(atan2(2.0f * (q[1] * q[2] + q[0] * q[3]),
                                 q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]))
                         * 180.0f / PI;  // 负号 = Petoi 的偏航符号约定
  float diff = yprHistory[index][0]
               - yprHistory[(index + ICM_MEAN_FILTER_SIZE - 1) % ICM_MEAN_FILTER_SIZE][0];
  // 偏航漂移补偿：原方案把"4 帧间隔 yaw 变化 < 0.1°"一律当漂移积分扣除，
  // 原理性缺陷是真实的慢速旋转（<5°/s，如慢速转弯）也会被吞掉。
  // 此处加静止门控：三轴角速度均 < 1°/s（即真的没在转）才积累漂移。
  // 若要回退到原行为，把 still 条件改成 true 即可。
  bool still = fabsf(_gyroDps[0]) < 1.0f && fabsf(_gyroDps[1]) < 1.0f
               && fabsf(_gyroDps[2]) < 1.0f;
  if (still && fabsf(diff) < 0.1f)
    yawDrift += diff;
  ypr[0] = yprHistory[index][0] - yawDrift;
  // pitch/roll 的 4 点滑动均值：预热期(前 4 帧)按已有样本数平均，之后全窗平均
  int8_t prevCount = firstRound ? index : ICM_MEAN_FILTER_SIZE;
  int8_t newCount = firstRound ? index + 1 : ICM_MEAN_FILTER_SIZE;
  ypr[1] = ypr[1] * prevCount - yprHistory[index][1];
  ypr[2] = ypr[2] * prevCount - yprHistory[index][2];
  float pitchArg = 2.0f * (q[1] * q[3] - q[0] * q[2]);
  if (pitchArg > 1.0f) pitchArg = 1.0f;    // 浮点误差可能越界，asin 域外是 NaN
  if (pitchArg < -1.0f) pitchArg = -1.0f;
  yprHistory[index][1] = (asin(pitchArg)) * 180.0f / PI;  if (az < 0)  // raw pitch 不会超过 90 度
    yprHistory[index][1] = (yprHistory[index][1] < 0 ? -1 : 1) * 180 - yprHistory[index][1];
  yprHistory[index][2] = (atan2(2.0f * (q[0] * q[1] + q[2] * q[3]),
                                q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]))
                         * 180.0f / PI;
  ypr[1] = (ypr[1] + yprHistory[index][1]) / newCount;
  ypr[2] = (ypr[2] + yprHistory[index][2]) / newCount;
  if (firstRound && newCount >= ICM_MEAN_FILTER_SIZE)
    firstRound = false;
  index = (index + 1) % ICM_MEAN_FILTER_SIZE;
}
