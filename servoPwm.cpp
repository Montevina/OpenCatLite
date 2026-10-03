#include <Arduino.h>
#include "board.h"
#include "servoPwm.h"

void servoPwmSetup() {
  // TODO: 参考 espServo.h servoSetup()/attachAllESPServos()
  //  12 个通道按 240Hz、10 位分辨率 attach；
  //  LEDC 通道-定时器映射避免和 buzzer 抢定时器（buzzer 用固定 1 个通道）
}

void servoPwmWrite(uint8_t servo, uint16_t pulseUs) {
  // TODO: 脉宽 -> 占空比计数 -> ledcWrite。原版经 Servo::write()，可简化为直接换算
  (void)servo; (void)pulseUs;
}

void servoPwmOff(uint8_t servo) { (void)servo; /* TODO: ledcWrite 0 占空比 */ }
void servoPwmAllOff() { /* TODO: 循环 servoPwmOff */ }
