/* ============================================================
   servoPwm.cpp —— ESP32 LEDC 舵机脉冲输出
   对照原版：src/espServo.h 的 attach 流程 + PetoiESP32Servo 的
   ESP32PWM 底层。原版靠库管理通道/定时器/角度换算，这里直接
   用 Arduino 核心的 ledc API，一个文件把三件事做完。

   分辨率选 16 位 @240Hz 的账：
     周期 1/240 = 4166.7µs，LEDC 计数 65536 步 → 步进 0.064µs；
     P1L 270° 档 = 2000µs/270° ≈ 7.4µs/° → 0.0086°/步，
     远小于舵机机械死区（约 0.5~1°），精度无损失。
   240Hz 是 Petoi 的非标设定（原版 SERVO_FREQ 240），别改成 50。
   ============================================================ */
#include <Arduino.h>
#include "board.h"
#include "servoPwm.h"

#define SERVO_PWM_BITS 16

// 脉宽 µs -> LEDC 占空比计数（uint64 防溢出：2500*240*65536 ≈ 3.9e10）
static inline uint32_t pulseToDuty(uint16_t pulseUs) {
  return (uint32_t)((uint64_t)pulseUs * SERVO_FREQ_HZ * (1ULL << SERVO_PWM_BITS) / 1000000ULL);
}

// 物理舵机号 -> LEDC 通道（分配见文件头：0~7→8~15，8~11→0~3）
static inline uint8_t channelOf(uint8_t servo) {
  return servo < 8 ? (uint8_t)(servo + 8) : (uint8_t)(servo - 8);
}

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
// 新核心(3.x)：按引脚自动分配通道
void servoPwmSetup() {
  for (uint8_t s = 0; s < PWM_NUM; s++) {
    if (!ledcAttach(SERVO_PIN[s], SERVO_FREQ_HZ, SERVO_PWM_BITS))
      Serial.printf("[servoPwm] attach fail: pin %u\n", SERVO_PIN[s]);
    ledcWrite(SERVO_PIN[s], 0);  // 初始无脉冲 = 松开
  }
}
void servoPwmWrite(uint8_t servo, uint16_t pulseUs) {
  if (servo >= PWM_NUM) return;
  ledcWrite(SERVO_PIN[servo], pulseToDuty(pulseUs));
}
void servoPwmOff(uint8_t servo) {
  if (servo >= PWM_NUM) return;
  ledcWrite(SERVO_PIN[servo], 0);
}
#else
// 旧核心(2.x)：显式通道（当前编译目标 2.0.12 走这段）
void servoPwmSetup() {
  for (uint8_t s = 0; s < PWM_NUM; s++) {
    uint8_t ch = channelOf(s);
    if (!ledcSetup(ch, SERVO_FREQ_HZ, SERVO_PWM_BITS))
      Serial.printf("[servoPwm] setup fail: ch %u\n", ch);
    ledcAttachPin(SERVO_PIN[s], ch);
    ledcWrite(ch, 0);  // 初始无脉冲 = 松开
  }
}
void servoPwmWrite(uint8_t servo, uint16_t pulseUs) {
  if (servo >= PWM_NUM) return;
  ledcWrite(channelOf(servo), pulseToDuty(pulseUs));
}
void servoPwmOff(uint8_t servo) {
  if (servo >= PWM_NUM) return;
  ledcWrite(channelOf(servo), 0);
}
#endif

void servoPwmAllOff() {
  for (uint8_t s = 0; s < PWM_NUM; s++) servoPwmOff(s);
}
