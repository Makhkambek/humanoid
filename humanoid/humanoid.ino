/*
 * Humanoid — 8 серво (hold position) + 2x BTS7960 (master/slave) + Encoder + PID
 *
 * Серво держат фиксированные позиции из INIT_ANGLES[].
 * Через Serial можно сменить угол любого серво или все сразу.
 *
 * Серво:
 *   GPIO 4, 5, 6, 7, 15, 16, 40, 41
 *
 * Моторы плеча (master/slave):
 *   BTS7960 #1 (master, с энкодером):
 *     GPIO 1  → R_PWM
 *     GPIO 2  → L_PWM
 *     GPIO 14 → R_EN + L_EN
 *   BTS7960 #2 (slave, инвертирован — стоит зеркально):
 *     GPIO 21 → R_PWM
 *     GPIO 38 → L_PWM
 *     GPIO 39 → R_EN + L_EN
 *
 * Encoder REV Core Hex (через voltage divider 5V→3.3V):
 *   GPIO 12 → A
 *   GPIO 13 → B
 *
 * Serial команды (115200, Newline):
 *   <pin> <angle>  — серво на угол (например: 4 90)
 *   all <angle>    — все серво на угол
 *   home           — все серво в INIT_ANGLES
 *   list           — показать текущие углы
 *   <num>          — мотор: target в градусах (50, -180)
 *   m<num>         — мотор raw power (m100, m-150, m0)
 *   e<num>         — set encoder count (e0 = сброс)
 *   x              — стоп мотор target
 *   p<num>         — kP
 *   i<num>         — kI
 *   d<num>         — kD
 *   ?              — показать PID
 */

#include <ESP32Servo.h>
#include <ESP32Encoder.h>

// ─── Серво ───────────────────────────────────────────────────────
const uint8_t SERVO_PINS[8]  = {  4,  5,  6,  7, 15, 16, 40, 41 };
const uint8_t INIT_ANGLES[8] = {  90,  0,  0,  0,  0,  0,  0,  0 };  // стартовые позиции
const uint8_t NUM_SERVOS = 8;

#define PULSE_MIN_US  500
#define PULSE_MAX_US  2500

Servo servos[NUM_SERVOS];
uint8_t currentAngles[NUM_SERVOS];

// ─── Моторы BTS7960 ──────────────────────────────────────────────
#define M1_RPWM   1
#define M1_LPWM   2
#define M1_EN    14
#define M2_RPWM  21
#define M2_LPWM  38
#define M2_EN    39
#define PWM_FREQ  20000
#define PWM_RES   8

const bool MOTOR1_REVERSED = false;
const bool MOTOR2_REVERSED = true;

int currentMotorPower = 0;

// ─── Энкодер ─────────────────────────────────────────────────────
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

// ─────────────────────────────────────────────────────────────────

void setMotors(int16_t power) {
  power = constrain(power, -255, 255);
  currentMotorPower = power;

  int16_t pw1 = MOTOR1_REVERSED ? -power : power;
  if (pw1 >= 0) { ledcWrite(M1_RPWM, pw1); ledcWrite(M1_LPWM, 0); }
  else          { ledcWrite(M1_RPWM, 0);   ledcWrite(M1_LPWM, -pw1); }

  int16_t pw2 = MOTOR2_REVERSED ? -power : power;
  if (pw2 >= 0) { ledcWrite(M2_RPWM, pw2); ledcWrite(M2_LPWM, 0); }
  else          { ledcWrite(M2_RPWM, 0);   ledcWrite(M2_LPWM, -pw2); }
}

void resetPid() {
  integralAcc = 0;
  prevError = 0;
  prevPidTime = millis();
}

int8_t findServoByPin(uint8_t pin) {
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    if (SERVO_PINS[i] == pin) return i;
  }
  return -1;
}

void setServoByPin(uint8_t pin, uint8_t angle) {
  int8_t idx = findServoByPin(pin);
  if (idx < 0) {
    Serial.print("[ERR] GPIO "); Serial.print(pin); Serial.println(" не найден");
    return;
  }
  angle = constrain(angle, 0, 180);
  servos[idx].write(angle);
  currentAngles[idx] = angle;
  Serial.print("[SERVO GPIO "); Serial.print(pin);
  Serial.print("] → "); Serial.print(angle); Serial.println("°");
}

void setAllServos(uint8_t angle) {
  angle = constrain(angle, 0, 180);
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    servos[i].write(angle);
    currentAngles[i] = angle;
  }
  Serial.print("[ALL SERVOS] → "); Serial.print(angle); Serial.println("°");
}

void homeServos() {
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    servos[i].write(INIT_ANGLES[i]);
    currentAngles[i] = INIT_ANGLES[i];
  }
  Serial.println("[HOME] серво в стартовых позициях");
}

void listAngles() {
  Serial.println("─── Серво ───");
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    Serial.print("  GPIO ");
    if (SERVO_PINS[i] < 10) Serial.print(" ");
    Serial.print(SERVO_PINS[i]);
    Serial.print(" → "); Serial.print(currentAngles[i]); Serial.println("°");
  }
  long count = encoder.getCount();
  Serial.print("─── Мотор: pwr="); Serial.print(currentMotorPower);
  Serial.print(", enc="); Serial.print(count);
  Serial.print(", deg="); Serial.println(count / COUNTS_PER_DEG, 1);
}

// ─────────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.println("\n=== Humanoid (hold position) ===");

  // Серво
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    servos[i].setPeriodHertz(50);
    servos[i].attach(SERVO_PINS[i], PULSE_MIN_US, PULSE_MAX_US);
    servos[i].write(INIT_ANGLES[i]);
    currentAngles[i] = INIT_ANGLES[i];
  }
  Serial.println("[SERVO] 8 серво init OK — держат позицию");

  // Моторы
  ledcAttach(M1_RPWM, PWM_FREQ, PWM_RES);
  ledcAttach(M1_LPWM, PWM_FREQ, PWM_RES);
  ledcAttach(M2_RPWM, PWM_FREQ, PWM_RES);
  ledcAttach(M2_LPWM, PWM_FREQ, PWM_RES);
  pinMode(M1_EN, OUTPUT); digitalWrite(M1_EN, HIGH);
  pinMode(M2_EN, OUTPUT); digitalWrite(M2_EN, HIGH);
  setMotors(0);
  Serial.println("[MOTOR] BTS7960 #1 + #2 init OK");

  // Encoder
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoder.attachFullQuad(ENC_A_PIN, ENC_B_PIN);
  encoder.clearCount();
  Serial.println("[ENC] init OK");

  listAngles();

  Serial.println("\nКоманды:");
  Serial.println("  4 90       серво GPIO 4 → 90°");
  Serial.println("  all 45     все серво → 45°");
  Serial.println("  home       все серво в стартовые позиции");
  Serial.println("  list       показать состояние");
  Serial.println("  <num>      мотор: target в градусах");
  Serial.println("  m<num>     мотор raw power (m100, m0)");
  Serial.println("  e<num>     set encoder (e0 = сброс)");
  Serial.println("  x          стоп мотор");
  Serial.println("  p/i/d<n>   тюнинг PID,   ?  показать PID");
}

void loop() {
  // ─── Команды ──────────────────────────────────────────────────
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;

    String cmd = line;
    cmd.toLowerCase();

    if (cmd == "home") {
      homeServos();
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
      setAllServos((uint8_t)constrain(angle, 0, 180));
    } else if (cmd[0] == 'm') {
      targetActive = false;
      setMotors(line.substring(1).toInt());
      Serial.print("[POWER] "); Serial.println(currentMotorPower);
    } else if (cmd[0] == 'e') {
      long v = line.substring(1).toInt();
      encoder.setCount(v);
      Serial.print("[ENC SET] "); Serial.println(v);
    } else if (cmd[0] == 'p') {
      kP = line.substring(1).toFloat(); Serial.print("[kP] "); Serial.println(kP);
    } else if (cmd[0] == 'i') {
      kI = line.substring(1).toFloat(); Serial.print("[kI] "); Serial.println(kI);
    } else if (cmd[0] == 'd') {
      kD = line.substring(1).toFloat(); Serial.print("[kD] "); Serial.println(kD);
    } else {
      // "<pin> <angle>" или "<degrees>" для мотора
      int spaceIdx = line.indexOf(' ');
      if (spaceIdx > 0) {
        int pin   = line.substring(0, spaceIdx).toInt();
        int angle = line.substring(spaceIdx + 1).toInt();
        setServoByPin((uint8_t)pin, (uint8_t)angle);
      } else if (isDigit(line[0]) || line[0] == '-' || line[0] == '+') {
        float deg = line.toFloat();
        targetCounts = (long)(deg * COUNTS_PER_DEG);
        targetActive = true;
        resetPid();
        Serial.print("[TARGET] "); Serial.print(deg);
        Serial.print("° = "); Serial.println(targetCounts);
      } else {
        Serial.print("[ERR] "); Serial.println(line);
      }
    }
  }

  // ─── PID для моторов ──────────────────────────────────────────
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
      if (power >  MAX_PW) power =  MAX_PW;
      if (power < -MAX_PW) power = -MAX_PW;
      if (power > 0 && power <  MIN_PW) power =  MIN_PW;
      if (power < 0 && power > -MIN_PW) power = -MIN_PW;

      setMotors(power);
      prevError = error;
      prevPidTime = now;
    }
  }

  // ─── Telemetry (только когда мотор активен) ───────────────────
  if (targetActive && (millis() - lastTelemetry > TELEMETRY_INTERVAL)) {
    long count = encoder.getCount();
    Serial.print("[T] pwr=");  Serial.print(currentMotorPower);
    Serial.print(" | enc=");   Serial.print(count);
    Serial.print(" | err=");   Serial.println(targetCounts - count);
    lastTelemetry = millis();
  }
}
