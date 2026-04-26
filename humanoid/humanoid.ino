/*
 * HumanoidTester — 8 серво (auto toggle) + 2x BTS7960 (master/slave) + Encoder + PID
 *
 * Серво (auto toggle 0° ↔ 90° каждые 3 сек):
 *   GPIO 4, 5, 6, 7, 15, 16, 17, 18
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
 * Encoder REV Core Hex (только на мастере, через voltage divider 5V→3.3V):
 *   GPIO 12 → A
 *   GPIO 13 → B
 *
 * Serial команды (115200, Newline):
 *   <num>     — target в градусах   (50, 360, -180)
 *   m<num>    — raw power           (m100, m-150, m0)
 *   e<num>    — set encoder count   (e0 = сброс)
 *   x         — стоп target
 *   p<num>    — kP
 *   i<num>    — kI
 *   d<num>    — kD
 *   ?         — показать PID
 *   sv<n>     — серво пауза/старт   (sv0 = stop, sv1 = run)
 */

#include <ESP32Servo.h>
#include <ESP32Encoder.h>

// ─── Серво ───────────────────────────────────────────────────────
const uint8_t SERVO_PINS[8] = { 4, 5, 6, 7, 15, 16, 40, 41 };
const uint8_t NUM_SERVOS = 8;
#define PULSE_MIN_US  500
#define PULSE_MAX_US  2500
Servo servos[NUM_SERVOS];

bool servoActive = true;
uint8_t servoAngle = 0;
unsigned long lastServoToggle = 0;
const unsigned long SERVO_INTERVAL = 3000;

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

// ─── Управление моторами ────────────────────────────────────────
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

void resetPid() {
  integralAcc = 0;
  prevError = 0;
  prevPidTime = millis();
}

void allServos(uint8_t a) {
  for (uint8_t i = 0; i < NUM_SERVOS; i++) servos[i].write(a);
}

void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.println("\n=== HumanoidTester ===");

  // Серво
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    servos[i].setPeriodHertz(50);
    servos[i].attach(SERVO_PINS[i], PULSE_MIN_US, PULSE_MAX_US);
  }
  allServos(0);
  Serial.println("[SERVO] 8 серво init OK");

  // Моторы
  ledcAttach(M1_RPWM, PWM_FREQ, PWM_RES);
  ledcAttach(M1_LPWM, PWM_FREQ, PWM_RES);
  ledcAttach(M2_RPWM, PWM_FREQ, PWM_RES);
  ledcAttach(M2_LPWM, PWM_FREQ, PWM_RES);
  pinMode(M1_EN, OUTPUT);  digitalWrite(M1_EN, HIGH);
  pinMode(M2_EN, OUTPUT);  digitalWrite(M2_EN, HIGH);
  setMotors(0);
  Serial.println("[MOTOR] BTS7960 #1 + #2 init OK");

  // Encoder
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoder.attachFullQuad(ENC_A_PIN, ENC_B_PIN);
  encoder.clearCount();
  Serial.println("[ENC] init OK");

  Serial.println("\nКоманды:");
  Serial.println("  <num>    target в degrees   (50, 360, -180)");
  Serial.println("  m<num>   raw power          (m-255..m255)");
  Serial.println("  e<num>   set encoder        (e0 = сброс)");
  Serial.println("  x        стоп target");
  Serial.println("  p/i/d <num>  тюнинг PID");
  Serial.println("  ?        показать PID");
  Serial.println("  sv<0|1>  серво pause/run    (sv0 / sv1)");
}

void loop() {
  // ─── Команды ──────────────────────────────────────────────────
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();

    if (line.length() == 0) {
      // skip
    } else if (line.startsWith("sv")) {
      servoActive = (line.substring(2).toInt() != 0);
      Serial.print("[SERVO] auto = "); Serial.println(servoActive ? "ON" : "OFF");
    } else if (line[0] == 'm') {
      targetActive = false;
      setMotors(line.substring(1).toInt());
      Serial.print("[POWER] "); Serial.println(currentMotorPower);
    } else if (line[0] == 'e') {
      long v = line.substring(1).toInt();
      encoder.setCount(v);
      Serial.print("[ENC SET] "); Serial.println(v);
    } else if (line[0] == 'x') {
      targetActive = false;
      setMotors(0);
      Serial.println("[STOP]");
    } else if (line[0] == 'p') {
      kP = line.substring(1).toFloat();  Serial.print("[kP] "); Serial.println(kP);
    } else if (line[0] == 'i') {
      kI = line.substring(1).toFloat();  Serial.print("[kI] "); Serial.println(kI);
    } else if (line[0] == 'd') {
      kD = line.substring(1).toFloat();  Serial.print("[kD] "); Serial.println(kD);
    } else if (line[0] == '?') {
      Serial.print("kP="); Serial.print(kP);
      Serial.print(" kI="); Serial.print(kI);
      Serial.print(" kD="); Serial.println(kD);
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

  // ─── Серво auto toggle ────────────────────────────────────────
  if (servoActive && (millis() - lastServoToggle > SERVO_INTERVAL)) {
    servoAngle = (servoAngle == 0) ? 90 : 0;
    allServos(servoAngle);
    lastServoToggle = millis();
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
      if (power > MAX_PW) power = MAX_PW;
      else if (power < -MAX_PW) power = -MAX_PW;
      if (power > 0 && power < MIN_PW) power = MIN_PW;
      else if (power < 0 && power > -MIN_PW) power = -MIN_PW;

      setMotors(power);
      prevError = error;
      prevPidTime = now;
    }
  }

  // ─── Telemetry ────────────────────────────────────────────────
  if (millis() - lastTelemetry > TELEMETRY_INTERVAL) {
    long count = encoder.getCount();
    float deg  = count / COUNTS_PER_DEG;

    Serial.print("[T] sv=");       Serial.print(servoAngle);
    Serial.print(" | pwr=");       Serial.print(currentMotorPower);
    Serial.print(" | enc=");       Serial.print(count);
    Serial.print(" | deg=");       Serial.print(deg, 1);
    if (targetActive) {
      Serial.print(" | tgt=");     Serial.print(targetCounts);
      Serial.print(" | err=");     Serial.print(targetCounts - count);
    }
    Serial.println();

    lastTelemetry = millis();
  }
}
