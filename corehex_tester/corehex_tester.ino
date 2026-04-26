/*
 * CoreHex Tester — ТЕСТ 2 СЕРВО (GPIO 17, 18), без моторов
 *
 * Просто проверка что серво на 17 и 18 работают.
 *
 * Подключение:
 *   GPIO 17 → signal серво 1
 *   GPIO 18 → signal серво 2
 *   V+ → внешний 5-6V
 *   GND → общий с ESP32
 *
 * Поведение: оба серво синхронно 0° ↔ 90° каждые 3 сек.
 *
 * Библиотека: ESP32Servo by Kevin Harrington
 */

#include <ESP32Servo.h>

#define SERVO1_PIN  17
#define SERVO2_PIN  18

#define PULSE_MIN_US  500
#define PULSE_MAX_US  2500

Servo servo1;
Servo servo2;

void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.println("\n=== CoreHex Tester — 2 серво (GPIO 17, 18) ===");

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  servo1.setPeriodHertz(50);
  servo1.attach(SERVO1_PIN, PULSE_MIN_US, PULSE_MAX_US);
  servo2.setPeriodHertz(50);
  servo2.attach(SERVO2_PIN, PULSE_MIN_US, PULSE_MAX_US);

  Serial.print("[INIT] GPIO ");
  Serial.print(SERVO1_PIN); Serial.print(" + ");
  Serial.println(SERVO2_PIN);
}

void loop() {
  servo1.write(0);
  servo2.write(0);
  Serial.println("[BOTH] → 0°");
  delay(3000);

  servo1.write(90);
  servo2.write(90);
  Serial.println("[BOTH] → 90°");
  delay(3000);
}
