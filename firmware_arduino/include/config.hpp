#ifndef H_CONFIG
#define H_CONFIG

#include <Arduino.h>

constexpr unsigned long SERIAL_BAUD = 115200;
constexpr int START_DELAY_MS = 100;

constexpr float ANGLE_PER_PULSE = PI / 18.0f;
constexpr unsigned long MIN_PULSE_PERIOD_US = 2000UL;
constexpr unsigned long STOP_TIMEOUT_US = 600000UL;

constexpr int RIGHT_TACH_PIN = 2;
constexpr int LEFT_TACH_PIN = 3;
constexpr int RIGHT_PWM_PIN = 10;
constexpr int LEFT_PWM_PIN = 9;

// 50 Hz
constexpr unsigned long CONTROL_PERIOD_MS = 20UL;
constexpr unsigned long CONTROL_PERIOD_US = CONTROL_PERIOD_MS * 1000UL;
constexpr float CONTROL_DT_SEC = CONTROL_PERIOD_MS / 1000.0f;

constexpr float DEFAULT_TARGET_VELOCITY = 0.0f; // rad/s
constexpr float KP = 27.0f;
constexpr float KI = 30.0f;
constexpr float K_FF = 40.0f;
constexpr float MIN_PWM_DEADZONE = 40.0f;
constexpr float MAX_INTEGRAL = 50.0f;
constexpr int MAX_PWM = 255;

#endif // H_CONFIG
