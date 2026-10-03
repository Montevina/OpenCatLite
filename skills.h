/* ============================================================
   skills —— L3 运动层：技能数据表与读取
   技能数据格式完全沿用原版（资产复用），一条技能 = 技能头 + 帧序列：
     period == 1   姿势：1 帧 x 16 关节
     period >  1   步态：period 帧 x WALKING_DOF(8) 关节
     period <  0   行为：|period| 帧 x (16+4)，行尾 4 字节 =
                    过渡速度/8、帧延时x50ms、IMU 触发轴、触发角
     头部：期望 pitch/roll、angleDataRatio(角度>125 时全帧除 2)、
           行为另有 loopCycle[3]（循环起止行、次数，负数=无限）
   与原版的区别：读出的帧拷贝进调用方缓冲，绝不把 dutyAngles
   指针别进命令缓冲区（原版 skill.h inplaceShift() 的别名技巧废弃）。
   挖数据：src/InstinctBittleESP.h 原样搬运（progmemPointer 表 + 数组）。
   挖解析：src/skill.h buildSkill()/formatSkill()（删 inplaceShift）。
   ============================================================ */
#ifndef SKILLS_H
#define SKILLS_H
#include <stdint.h>

struct SkillInfo {
  const char *name;   // 名字末字符约定同原版：'F' 步行 'L' 左 'R' 右(镜像)
  int period;         // 见上
  uint8_t ratio;      // angleDataRatio
};

void skillsInit();
int skillsFind(const char *name);        // 精确/方向匹配，找不到 -1（参考原版 lookUp）
bool skillsLoad(int index);              // 载入模块内部帧缓冲
const SkillInfo *skillsInfo();
uint8_t skillsFrameCount();              // |period|
uint8_t skillsFrameSize();               // 16 / 8 / 20
// 取第 frame 帧，已乘 angleDataRatio 还原成角度；返回 false = 越界
bool skillsReadFrame(uint8_t frame, float *outAngles);

#endif
