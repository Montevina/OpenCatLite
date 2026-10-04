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
  for (int h = 0; h < 4; h++)
    yprHistory[h][0] = yprHistory[h][1] = yprHistory[h][2] = 0;
  ax_real = ay_real = az_real = gx_real = gy_real = gz_real = 0;
  for (int i = 0; i < 3; i++) {
    a_real[i] = ypr[i] = 0;
    offset_accel[i] = offset_gyro[i] = 0;
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

// ---- 以下数据处理与融合为原 petoi_icm42670p 代码，除注明处未改动 ----

void ICM42670Lite::transformIMUDataWithOffset() {
  a_real[0] = ax_real = (_accelRaw[0] - offset_accel[0]) / accel_ratio;
  a_real[1] = ay_real = (_accelRaw[1] - offset_accel[1]) / accel_ratio;
  a_real[2] = az_real = (_accelRaw[2] - offset_accel[2]) / accel_ratio;
  gx_real = (_gyroRaw[0] - offset_gyro[0]) / gyro_ratio;
  gy_real = (_gyroRaw[1] - offset_gyro[1]) / gyro_ratio;
  gz_real = (_gyroRaw[2] - offset_gyro[2]) / gyro_ratio;
}

void ICM42670Lite::getOffset(int num) {
  Serial.println("Start IMU calibration...");
  Serial.println("Please put the sensor on a leveled plane!");
  Serial.println("Calculate mean");
  for (int i = 0; i < num; i++) {
    readData();
    offset_accel[0] += _accelRaw[0];
    offset_accel[1] += _accelRaw[1];
    offset_accel[2] += _accelRaw[2];
    offset_gyro[0] += _gyroRaw[0];
    offset_gyro[1] += _gyroRaw[1];
    offset_gyro[2] += _gyroRaw[2];
    delay(5);  // 200Hz ODR 下 5ms 间隔保证读到新样本
  }

  offset_accel[0] = offset_accel[0] / num;
  offset_accel[1] = offset_accel[1] / num;
  offset_accel[2] = offset_accel[2] / num - accel_ratio;  // Z 轴带 1g 重力
  offset_gyro[0] = offset_gyro[0] / num;
  offset_gyro[1] = offset_gyro[1] / num;
  offset_gyro[2] = offset_gyro[2] / num;
}

void ICM42670Lite::getImuGyro() {
  if (!readData()) return;  // 无新数据（原版靠 imuData 旧值比较间接实现同样的跳过）
  if (_accelRaw[0] != _prevAccelRaw[0] || _accelRaw[1] != _prevAccelRaw[1]
      || _accelRaw[2] != _prevAccelRaw[2]) {
    for (int i = 0; i < 3; i++) _prevAccelRaw[i] = _accelRaw[i];
    transformIMUDataWithOffset();
    uint32_t now = micros();
    deltaT = ((now - lastUpdate) / 1000000.0f);
    lastUpdate = now;
    // 陀螺速率转 rad/s 后进融合
    MadgwickQuaternionUpdate(ax_real, ay_real, az_real, gx_real * PI / 180.0f,
                             gy_real * PI / 180.0f, gz_real * PI / 180.0f, deltaT);
  }
}

// Sebastian Madgwick 定向滤波（原 petoi_icm42670p 实现，未改动；
// 含 4 点均值滤波、偏航漂移补偿、az<0 时的俯仰翻转）
void ICM42670Lite::MadgwickQuaternionUpdate(float ax, float ay, float az, float gyrox,
                                            float gyroy, float gyroz, float deltaT) {
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];         // short name local variable for readability
  float norm;                                               // vector norm
  float f1, f2, f3;                                         // objetive funcyion elements
  float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
  float qDot1, qDot2, qDot3, qDot4;
  float hatDot1, hatDot2, hatDot3, hatDot4;
  float gerrx, gerry, gerrz;                                // gyro bias error
  static float gbiasx = 0.0f, gbiasy = 0.0f, gbiasz = 0.0f; // gyro bias (static to maintain state)

  float GyroMeasError = PI * (40.0f / 180.0f);    // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
  float beta = sqrt(3.0f / 4.0f) * GyroMeasError; // compute beta
  float GyroMeasDrift = PI * (2.0f / 180.0f);     // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
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
  gbiasx += gerrx * deltaT * zeta;
  gbiasy += gerry * deltaT * zeta;
  gbiasz += gerrz * deltaT * zeta;
  gyrox -= gbiasx;
  gyroy -= gbiasy;
  gyroz -= gbiasz;

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

  // transform quaternion to euler
  yprHistory[index][0] = -(atan2(2.0f * (q[1] * q[2] + q[0] * q[3]),
                                 q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]))
                         * 180.0f / PI;
  float diff = yprHistory[index][0]
               - yprHistory[(index + 4 - 1) % 4][0];  // MEAN_FILTER_SIZE=4；避免负数取模
  if (abs(diff) < 0.1)
    yawDrift += diff;
  ypr[0] = yprHistory[index][0] - yawDrift;
  int8_t prevCount = firstRound ? index : 4;
  int8_t newCount = firstRound ? index + 1 : 4;
  ypr[1] = ypr[1] * prevCount - yprHistory[index][1];
  ypr[2] = ypr[2] * prevCount - yprHistory[index][2];
  yprHistory[index][1] = (asin(2.0f * (q[1] * q[3] - q[0] * q[2]))) * 180.0f / PI;
  if (az < 0)  // raw pitch 不会超过 90 度
    yprHistory[index][1] = (yprHistory[index][1] < 0 ? -1 : 1) * 180 - yprHistory[index][1];
  yprHistory[index][2] = (atan2(2.0f * (q[0] * q[1] + q[2] * q[3]),
                                q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]))
                         * 180.0f / PI;
  ypr[1] = (ypr[1] + yprHistory[index][1]) / newCount;
  ypr[2] = (ypr[2] + yprHistory[index][2]) / newCount;
  if (firstRound && newCount >= 4)
    firstRound = false;
  index = (index + 1) % 4;
}
