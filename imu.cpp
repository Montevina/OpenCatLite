#include <Arduino.h>
#include "board.h"
#include "imu.h"

// TODO: 内部状态：portMUX_TYPE 快照锁 + 双缓冲；core0 任务句柄
// TODO: 挖原版 src/imu.h：
//   icm42670Setup()        -> 初始化序列
//   readIMU()              -> 读原始数据 + 融合成 ypr（FIFO/采样周期同原版）
//   getImuException()      -> 异常判定阈值表（thresX/Y/Z，走路/站立两档）
//   calibrateICM()         -> 标定流程（offset_gyro 读数 -32768 的容错也要搬）

void imuSetup(bool calibrateNow) { (void)calibrateNow; /* TODO */ }
bool imuGetSnapshot(ImuSnapshot *out) { (void)out; return false; /* TODO */ }
void imuCalibrate() { /* TODO: 标完调 storeWriteImuOffset */ }
void imuStop() { /* TODO: 置停止标志 + 等任务退出（参考原版 gc 任务的退出等待写法）*/ }
