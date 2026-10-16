/* ============================================================
   skills.cpp —— 技能数据表实现（第一切片：sit / calib 两个静态姿势）
   数据逐字节搬自原版：
     calib → InstinctBittleESP.h（X 与 R 相同：全零 = 零点/装配姿态）
     sit   → X 用 InstinctBittleESP.h、R 用 InstinctBittleESP_arm.h
             （差异只在 joint 2：X 是尾巴 -45°，R 是夹爪保持 0°）
   格式：4 字节头 [period, expectedRoll, expectedPitch, angleRatio]
   + |period| × 帧（静态姿势 = 1 帧 × 16 关节）。
   ============================================================ */
#include <Arduino.h>
#include "board.h"
#include "skills.h"

#define SKILL_HEADER 4
#define POSTURE_DOF 16

// calib：全零，即零点姿态（用于 'c' 校准）
static const int8_t DATA_calib[] = {
    1, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
};

#if defined(BITTLE_R)
// Bittle R：joint 2 = 头部夹爪，sit 时保持中立（原版 _arm.h）
static const int8_t DATA_sit[] = {
    1, 0, -30, 1,
    0, 0, 0, 0, -5, -5, 20, 20, 45, 45, 105, 105, 45, 45, -45, -45,
};
#else
// Bittle X：joint 2 = 尾巴，sit 时垂尾 -45°（原版 InstinctBittleESP.h）
static const int8_t DATA_sit[] = {
    1, 0, -30, 1,
    0, 0, -45, 0, -5, -5, 20, 20, 45, 45, 105, 105, 45, 45, -45, -45,
};
#endif

struct SkillDef {
  const char *name;
  const int8_t *data;
};
static const SkillDef SKILLS[] = {
    {"calib", DATA_calib},
    {"sit", DATA_sit},
};
#define SKILL_COUNT ((int)(sizeof(SKILLS) / sizeof(SKILLS[0])))

// ---- 载入状态（下一片扩展为多帧缓冲）----
static SkillInfo sInfo;
static int8_t sFrame[POSTURE_DOF];
static bool sLoaded = false;

void skillsInit() {
  Serial.printf("[skills] %d postures:", SKILL_COUNT);
  for (int i = 0; i < SKILL_COUNT; i++) Serial.printf(" %s", SKILLS[i].name);
  Serial.println();
}

int skillsFind(const char *name) {
  for (int i = 0; i < SKILL_COUNT; i++)
    if (!strcmp(SKILLS[i].name, name)) return i;
  return -1;
}

bool skillsLoad(int index) {
  if (index < 0 || index >= SKILL_COUNT) return false;
  const int8_t *d = SKILLS[index].data;
  if (d[0] != 1) return false;  // 本切片只支持单帧静态姿势
  sInfo.name = SKILLS[index].name;
  sInfo.period = d[0];
  sInfo.ratio = (uint8_t)d[3];
  for (int i = 0; i < POSTURE_DOF; i++) sFrame[i] = d[SKILL_HEADER + i];
  sLoaded = true;
  return true;
}

const SkillInfo *skillsInfo() {
  return sLoaded ? &sInfo : nullptr;
}

uint8_t skillsFrameCount() {
  return sLoaded ? 1 : 0;
}

uint8_t skillsFrameSize() {
  return sLoaded ? POSTURE_DOF : 0;
}

bool skillsReadFrame(uint8_t frame, float *outAngles) {
  if (!sLoaded || frame != 0) return false;
  for (int i = 0; i < POSTURE_DOF; i++) outAngles[i] = sFrame[i] * sInfo.ratio;
  return true;
}
