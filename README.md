# Humanoid Robot — ESP32-S3

Arduino-based humanoid robot project. Goal: solve calculator equations by physically pressing buttons with servo-driven fingers.

## Hardware

- **MCU:** ESP32-S3-DevKitC-1 (Dual-core LX7 @ 240 MHz, WiFi/BLE)
- **Servos:** 10× 20kg digital servos (arms + fingers)
- **Motors:** 2× REV Core Hex Motor (shoulder)
- **Drivers:** 2× BTS7960 (43A H-bridge)
- **Encoder:** REV Core Hex built-in quadrature (master motor only)
- **Power:** LiPo 12V → Buck 12V→6V (servos) + Buck 12V→5V (ESP)

## Sketches

### `humanoid/`
Full integration: 10 servos (auto-toggle) + 2 motors (master/slave with PID) + encoder + Serial control.

### `servotester/`
Same as humanoid but with all servos + motor PID together for testing.

### `corehex_tester/`
Isolated test for 2 servos on GPIO 17, 18 (no motors, no other servos). Used for debugging individual servo pins.

## Pin Map

| GPIO | Назначение |
|------|------------|
| 1, 2 | BTS7960 #1 R_PWM, L_PWM (master) |
| 14 | BTS7960 #1 R_EN + L_EN |
| 4-7, 15-18, 40, 41 | 10 серво signal |
| 12, 13 | Encoder A, B (через voltage divider 5V→3.3V) |
| 21, 38 | BTS7960 #2 R_PWM, L_PWM (slave) |
| 39 | BTS7960 #2 R_EN + L_EN |

## Serial Commands (115200 baud, Newline)

| Команда | Действие |
|---------|----------|
| `<num>` | target в градусах для мотора (P-I-D) |
| `m<num>` | raw power моторов (-255..255) |
| `e<num>` | set encoder count |
| `x` | стоп target tracking |
| `p<num>` | tune kP |
| `i<num>` | tune kI |
| `d<num>` | tune kD |
| `?` | show PID values |
| `sv0` / `sv1` | servo auto-toggle pause/run |

## Libraries

- `ESP32Servo` by Kevin Harrington
- `ESP32Encoder` by Kevin Harrington

Install via Arduino IDE Library Manager.

## Status

Pre-alpha. Servo control working, motor control with PID in tuning phase.
