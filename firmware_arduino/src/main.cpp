#include <Arduino.h>
#include <math.h>
#include "config.hpp"

volatile unsigned long right_last_pulse_ms = 0;
volatile unsigned long left_last_pulse_ms = 0;

volatile unsigned long right_delta_ms = 0;
volatile unsigned long left_delta_ms = 0;

float right_angular_velocity = 0;
float left_angular_velocity = 0;

constexpr float ANGLE_PER_PULSE = PI / 18.0;

void update_right_angular_velocity()
{
  unsigned long current_ms = millis();

  right_delta_ms = current_ms - right_last_pulse_ms;
  right_last_pulse_ms = current_ms;
}

void update_left_angular_velocity()
{
  unsigned long current_ms = millis();

  left_delta_ms = current_ms - left_last_pulse_ms;
  left_last_pulse_ms = current_ms;
}

void attach_interrupt_tachometer()
{
  attachInterrupt(
      digitalPinToInterrupt(right_tachometer_pin),
      update_right_angular_velocity,
      RISING);

  attachInterrupt(
      digitalPinToInterrupt(left_tachometer_pin),
      update_left_angular_velocity,
      RISING);
}

void setup()
{
  Serial.begin(115200);

  pinMode(right_tachometer_pin, INPUT);
  pinMode(left_tachometer_pin, INPUT);

  attach_interrupt_tachometer();

  delay(start_delay);
}

void loop()
{
  unsigned long right_dt;
  unsigned long left_dt;

  noInterrupts();
  right_dt = right_delta_ms;
  left_dt = left_delta_ms;
  interrupts();

  if (right_dt > 0)
    right_angular_velocity = ANGLE_PER_PULSE / (right_dt / 1000.0);

  if (left_dt > 0)
    left_angular_velocity = ANGLE_PER_PULSE / (left_dt / 1000.0);

  Serial.print("Right: ");
  Serial.print(right_angular_velocity);
  Serial.print(" rad/s | Left: ");
  Serial.println(left_angular_velocity);

  delay(10);
}