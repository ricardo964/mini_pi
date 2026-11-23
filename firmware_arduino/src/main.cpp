#include <Arduino.h>
#include "config.hpp"
#include "wheel.hpp"

struct Telemetry
{
  float left_wheel_velocity;
  float right_wheel_velocity;
};

struct Command
{
  float left_wheel_setpoint;
  float right_wheel_setpoint;
};

Command command;
Telemetry telemetry;
Wheel right_wheel;
Wheel left_wheel;

void isr_right_tachometer() {
  right_wheel.tach_isr();
}

void isr_left_tachometer() {
  left_wheel.tach_isr();
}

unsigned long next_control_us = 0;
unsigned long next_command_us = 0;
unsigned long next_telemetry_us = 0;

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
  next_command_us = micros();
  next_telemetry_us = micros();
}

void loop() {
  unsigned long now_us = micros();

  // Control loop period
  if (now_us - next_control_us >= CONTROL_PERIOD_US) {
    next_control_us += CONTROL_PERIOD_US;

    if (now_us - next_control_us > CONTROL_PERIOD_US * 4) {
      next_control_us = now_us;
    }

    right_wheel.update(CONTROL_DT_SEC);
    left_wheel.update(CONTROL_DT_SEC);

    telemetry.left_wheel_velocity = left_wheel.current_velocity;
    telemetry.right_wheel_velocity = right_wheel.current_velocity;
  }

  // Read command structure period
  if (now_us - next_command_us >= COMMAND_PERIOD_US) {
    next_command_us += COMMAND_PERIOD_US;

    if (now_us - next_command_us > COMMAND_PERIOD_US * 4) {
      next_command_us = now_us;
    }

    if (Serial.available() >= (int)sizeof(Command)) {
      Serial.readBytes(reinterpret_cast<char*>(&command), sizeof(command));

      // Use get_setpoint for each wheel from the command
      left_wheel.get_setpoint(command.left_wheel_setpoint);
      right_wheel.get_setpoint(command.right_wheel_setpoint);
    }
  }

  // Write telemetry structure period
  if (now_us - next_telemetry_us >= TELEMETRY_PERIOD_US) {
    next_telemetry_us += TELEMETRY_PERIOD_US;

    if (now_us - next_telemetry_us > TELEMETRY_PERIOD_US * 4) {
      next_telemetry_us = now_us;
    }

    Serial.write(0xAA);

    Serial.write(
      reinterpret_cast<const uint8_t*>(&telemetry),
      sizeof(telemetry)
    );

    Serial.write(0x55);
  }
}
