/*
 * ServoTester — кастомный код для ESP32 Arduino core 3.x
 * Использует ТОЛЬКО LEDC (без MCPWM, без сторонних libs кроме ESP32Encoder)
 *
 * ВАЖНО: ESP32-S3 имеет 8 LEDC каналов.
 *        4 серво (50Hz) + 4 motor PWM (20kHz) = 8 каналов = РОВНО хватает.
 *
 *        Чтобы разблокировать оставшиеся 4 серво — купи PCA9685.
 *
 * Активные серво: GPIO 4, 5, 6, 7
 * Моторы:         BTS7960 master/slave + REV Core Hex encoder
 *
 * Стартовые позиции:
 *   GPIO 6 → 0°
 *   Остальные → 0°
 *
 * Команды (115200, Newline):
 *   <pin> <angle>   серво на угол (4 60, 7 90)
 *   all <angle>     все серво на угол
 *   home            все на 0°
 *   list            показать состояние
 *   t <deg>         мотор → град (PID)
 *   m <pwr>         мотор raw power
 *   e <n>           set encoder count
 *   x               стоп target
 *   p/i/d <n>       тюнинг PID
 *   ?               показать PID
 */

#include <ESP32Encoder.h>

// ─── Серво — LEDC 50Hz ───────────────────────────────────────────
const uint8_t  SERVO_PINS[]  = {  4,  5,  6,  7 };
const uint8_t  INIT_ANGLES[] = {  0,  0,  0,  0 };
const uint8_t  NUM_SERVOS = sizeof(SERVO_PINS) / sizeof(SERVO_PINS[0]);

const int SERVO_FREQ = 50;
const int SERVO_RES = 16;
const uint32_t SERVO_DUTY_MAX = (1UL << SERVO_RES) - 1;

uint8_t currentAngles[NUM_SERVOS] = {0};

void servoSetup(uint8_t pin) {
  ledcAttach(pin, SERVO_FREQ, SERVO_RES);
}

void servoWrite(uint8_t pin, uint8_t angle) {
  angle = constrain((int)angle, 0, 180);
  uint32_t pulseUs = map(angle, 0, 180, 500, 2500);
  uint32_t duty = (uint32_t)((uint64_t)pulseUs * SERVO_DUTY_MAX / 20000UL);
  ledcWrite(pin, duty);
}

// ─── Моторы — LEDC 20kHz ─────────────────────────────────────────
#define M1_RPWM   1
#define M1_LPWM   2
#define M1_EN    14
#define M2_RPWM  21
#define M2_LPWM  38
#define M2_EN    39
#define MOTOR_FREQ  20000
#define MOTOR_RES   8

const bool MOTOR1_REVERSED = false;
const bool MOTOR2_REVERSED = true;
int currentMotorPower = 0;

void setMotors(int16_t power) {
  power = constrain(power, -255, 255);
  currentMotorPower = power;

  int16_t pw1 = MOTOR1_REVERSED ? -power : power;
  if (pw1 >= 0) { ledcWrite(M1_RPWM, pw1); ledcWrite(M1_LPWM, 0); }
  else          { ledcWrite(M1_RPWM, 0); ledcWrite(M1_LPWM, -pw1); }

  int16_t pw2 = MOTOR2_REVERSED ? -power : power;
  if (pw2 >= 0) { ledcWrite(M2_RPWM, pw2); ledcWrite(M2_LPWM, 0); }
  else          { ledcWrite(M2_RPWM, 0); ledcWrite(M2_LPWM, -pw2); }
}

// ─── Encoder ─────────────────────────────────────────────────────
#define ENC_A_PIN  12
#define ENC_B_PIN  13
ESP32Encoder encoder;
const float COUNTS_PER_REV = 1152.0;
const float COUNTS_PER_DEG = COUNTS_PER_REV / 360.0;

// ─── PID ─────────────────────────────────────────────────────────
float kP = 1.5;
float kI = 0.0;
float kD = 0.05;
bool  targetActive  = false;
long  targetCounts  = 0;
float integralAcc   = 0;
long  prevError     = 0;
unsigned long prevPidTime = 0;

const int   TOL    = 10;
const int   MIN_PW = 60;
const int   MAX_PW = 220;
const float I_LIMIT = 200;

// ─── Telemetry ───────────────────────────────────────────────────
unsigned long lastTelemetry = 0;
const unsigned long TELEMETRY_INTERVAL = 500;

// ─── Helpers ─────────────────────────────────────────────────────
int8_t findServoByPin(uint8_t pin) {
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    if (SERVO_PINS[i] == pin) return i;
  }
  return -1;
}

void setServoByPin(uint8_t pin, uint8_t angle) {
  int8_t idx = findServoByPin(pin);
  if (idx < 0) {
    Serial.print("[ERR] GPIO "); Serial.print(pin);
    Serial.println(" не подключён как серво");
    return;
  }
  servoWrite(pin, angle);
  currentAngles[idx] = angle;
  Serial.print("[GPIO "); Serial.print(pin);
  Serial.print("] → "); Serial.print(angle); Serial.println("°");
}

void setAllServos(uint8_t angle) {
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    servoWrite(SERVO_PINS[i], angle);
    currentAngles[i] = angle;
  }
  Serial.print("[ALL SERVOS] → "); Serial.print(angle); Serial.println("°");
}

void listAngles() {
  Serial.println("─── Серво ───");
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    Serial.print("  GPIO ");
    if (SERVO_PINS[i] < 10) Serial.print(" ");
    Serial.print(SERVO_PINS[i]);
    Serial.print(" → "); Serial.print(currentAngles[i]); Serial.println("°");
  }
  Serial.print("─── Мотор: pwr="); Serial.print(currentMotorPower);
  Serial.print(", enc="); Serial.println(encoder.getCount());
}

void resetPid() {
  integralAcc = 0;
  prevError = 0;
  prevPidTime = millis();
}

void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.println("\n=== ServoTester (LEDC only, 4 servos + 2 motors) ===");

  // Серво — LEDC 50Hz (каналы 0-3)
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    servoSetup(SERVO_PINS[i]);
    servoWrite(SERVO_PINS[i], INIT_ANGLES[i]);
    currentAngles[i] = INIT_ANGLES[i];
  }
  Serial.print("[SERVO] "); Serial.print(NUM_SERVOS);
  Serial.println(" серво init OK");

  // Encoder — ДО listAngles чтобы getCount() не был на neинициал
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoder.attachFullQuad(ENC_A_PIN, ENC_B_PIN);
  encoder.clearCount();
  Serial.println("[ENC] init OK");

  // Моторы — LEDC 20kHz (каналы 4-7)
  ledcAttach(M1_RPWM, MOTOR_FREQ, MOTOR_RES);
  ledcAttach(M1_LPWM, MOTOR_FREQ, MOTOR_RES);
  ledcAttach(M2_RPWM, MOTOR_FREQ, MOTOR_RES);
  ledcAttach(M2_LPWM, MOTOR_FREQ, MOTOR_RES);
  pinMode(M1_EN, OUTPUT);  digitalWrite(M1_EN, HIGH);
  pinMode(M2_EN, OUTPUT);  digitalWrite(M2_EN, HIGH);
  setMotors(0);
  Serial.println("[MOTOR] BTS7960 master/slave init OK");

  listAngles();

  Serial.println("\nКоманды:");
  Serial.println("  4 60       серво GPIO 4 → 60°");
  Serial.println("  all 90     все серво → 90°");
  Serial.println("  home       все серво → 0°");
  Serial.println("  list       показать состояние");
  Serial.println("  t 90       мотор → 90° (PID)");
  Serial.println("  m 100      мотор raw power");
  Serial.println("  e 0        сброс encoder");
  Serial.println("  x          стоп target");
  Serial.println("  p/i/d <n>  тюнинг PID, ?  показать PID");
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      line.replace('-', ' ');
      line.replace(';', ' ');
      line.replace(',', ' ');
      line.replace(':', ' ');
      while (line.indexOf("  ") >= 0) line.replace("  ", " ");
      line.trim();

      String cmd = line;
      cmd.toLowerCase();

      if (cmd == "home") {
        setAllServos(0);
      } else if (cmd == "list") {
        listAngles();
      } else if (cmd == "x") {
        targetActive = false;
        setMotors(0);
        Serial.println("[STOP]");
      } else if (cmd == "?") {
        Serial.print("kP="); Serial.print(kP);
        Serial.print(" kI="); Serial.print(kI);
        Serial.print(" kD="); Serial.println(kD);
      } else if (cmd.startsWith("all ")) {
        int angle = cmd.substring(4).toInt();
        if (angle >= 0 && angle <= 180) setAllServos((uint8_t)angle);
        else Serial.println("[ERR] угол 0-180");
      } else if (cmd.startsWith("t ")) {
        float deg = cmd.substring(2).toFloat();
        targetCounts = (long)(deg * COUNTS_PER_DEG);
        targetActive = true;
        resetPid();
        Serial.print("[TARGET] "); Serial.print(deg);
        Serial.print("° = "); Serial.println(targetCounts);
      } else if (cmd.startsWith("m ")) {
        targetActive = false;
        setMotors(cmd.substring(2).toInt());
        Serial.print("[POWER] "); Serial.println(currentMotorPower);
      } else if (cmd.startsWith("e ")) {
        long v = cmd.substring(2).toInt();
        encoder.setCount(v);
        Serial.print("[ENC SET] "); Serial.println(v);
      } else if (cmd.startsWith("p ")) {
        kP = cmd.substring(2).toFloat();  Serial.print("[kP] "); Serial.println(kP);
      } else if (cmd.startsWith("i ")) {
        kI = cmd.substring(2).toFloat();  Serial.print("[kI] "); Serial.println(kI);
      } else if (cmd.startsWith("d ")) {
        kD = cmd.substring(2).toFloat();  Serial.print("[kD] "); Serial.println(kD);
      } else {
        int spaceIdx = cmd.indexOf(' ');
        if (spaceIdx < 0) {
          Serial.print("[ERR] неизвестная команда: "); Serial.println(line);
          return;
        }
        int pin = cmd.substring(0, spaceIdx).toInt();
        int angle = cmd.substring(spaceIdx + 1).toInt();
        if (angle < 0 || angle > 180) {
          Serial.println("[ERR] угол 0-180");
          return;
        }
        setServoByPin((uint8_t)pin, (uint8_t)angle);
      }
    }
  }

  // PID
  if (targetActive) {
    long current = encoder.getCount();
    long error = targetCounts - current;

    if (abs(error) < TOL) {
      setMotors(0);
      targetActive = false;
      Serial.print("[REACHED] target="); Serial.print(targetCounts);
      Serial.print(" current="); Serial.println(current);
    } else {
      unsigned long now = millis();
      float dt = (now - prevPidTime) / 1000.0f;
      if (dt < 0.001f) dt = 0.001f;

      integralAcc += error * dt;
      integralAcc = constrain(integralAcc, -I_LIMIT, I_LIMIT);
      float derivative = (error - prevError) / dt;

      float output = kP * error + kI * integralAcc + kD * derivative;

      int power = (int)output;
      if (power > MAX_PW) power = MAX_PW;
      else if (power < -MAX_PW) power = -MAX_PW;
      if (power > 0 && power < MIN_PW) power = MIN_PW;
      else if (power < 0 && power > -MIN_PW) power = -MIN_PW;

      setMotors(power);
      prevError = error;
      prevPidTime = now;
    }
  }

  // Telemetry
  if (targetActive && (millis() - lastTelemetry > TELEMETRY_INTERVAL)) {
    long count = encoder.getCount();
    Serial.print("[T] pwr=");      Serial.print(currentMotorPower);
    Serial.print(" | enc=");       Serial.print(count);
    Serial.print(" | err=");       Serial.println(targetCounts - count);
    lastTelemetry = millis();
  }
}
