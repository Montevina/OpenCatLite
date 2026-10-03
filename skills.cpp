#include <Arduino.h>
#include "skills.h"

// TODO: 技能数据表按机型二选一：
//   Bittle X <- InstinctBittleESP.h    Bittle R <- InstinctBittleESP_arm.h
// TODO: Bittle R 的查找规则（原版 loadBySkillName 的 ROBOT_ARM 分支）：
//   方向性技能(末字符 F/L/R)先试 "去掉末字符+Arm+末字符" 的变体
//   （如 wkF -> wkArmF），命中则用臂优化步态，否则回退普通版
// TODO: 'R' 镜像逻辑照搬原版 mirror()，但 Bittle R 不镜像 2 号关节(夹爪)
// TODO: Bittle R 载入姿势/非 Arm 优化步态后调重心补偿：肩俯仰 0/1 列
//   +角度、4/5 列 -角度x1.2（原版 shiftCenterOfMass(-10)），补偿臂的重量
// TODO: 模块内部静态帧缓冲（最大：行为 |period| x 20，约 2.5KB，
//   对应原版 BUFF_LEN 的容量计算），skillsLoad 做值拷贝

void skillsInit() { /* TODO */ }
int skillsFind(const char *name) { (void)name; return -1; /* TODO */ }
bool skillsLoad(int index) { (void)index; return false; /* TODO */ }
const SkillInfo *skillsInfo() { return nullptr; /* TODO */ }
uint8_t skillsFrameCount() { return 0; /* TODO */ }
uint8_t skillsFrameSize() { return 0; /* TODO */ }
bool skillsReadFrame(uint8_t frame, float *outAngles) {
  (void)frame; (void)outAngles; return false;  // TODO
}
