#include <Arduino.h>
#include "config.hpp"
#include "wheel.hpp"

Wheel right_wheel;
Wheel left_wheel;

void isr_right_tachometer() {
  right_wheel.tach_isr();
}

void isr_left_tachometer() {
  left_wheel.tach_isr();
}

unsigned long next_control_us = 0;

void setup() {
  Serial.begin(SERIAL_BAUD);

  right_wheel.init(RIGHT_PWM_PIN, RIGHT_TACH_PIN, DEFAULT_TARGET_VELOCITY);
  left_wheel.init(LEFT_PWM_PIN, LEFT_TACH_PIN, DEFAULT_TARGET_VELOCITY);

  attachInterrupt(
      digitalPinToInterrupt(RIGHT_TACH_PIN),
      isr_right_tachometer,
      RISING);

  attachInterrupt(
      digitalPinToInterrupt(LEFT_TACH_PIN),
      isr_left_tachometer,
      RISING);

  delay(START_DELAY_MS);

  next_control_us = micros();
}

void loop() {
  unsigned long now_us = micros();

  if (now_us - next_control_us >= CONTROL_PERIOD_US) {
    next_control_us += CONTROL_PERIOD_US;

    if (now_us - next_control_us > CONTROL_PERIOD_US * 4) {
      next_control_us = now_us;
    }

    right_wheel.update(CONTROL_DT_SEC);
    left_wheel.update(CONTROL_DT_SEC);

    // Serial.print(right_wheel.current_velocity, 2);
    // Serial.print(", ");
    // Serial.print(right_wheel.control_output);
    // Serial.print(", ");
    // Serial.print(left_wheel.current_velocity, 2);
    // Serial.print(", ");
    // Serial.println(left_wheel.control_output);
  }
}
