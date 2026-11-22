#ifndef H_WHEEL
#define H_WHEEL

struct Wheel {
  int pwm_pin = -1;
  int tach_pin = -1;

  volatile unsigned long last_pulse_us = 0;
  volatile unsigned long delta_us = 0;

  float target_velocity = 0.0f;
  float current_velocity = 0.0f;
  float integral_error = 0.0f;
  int control_output = 0;

  void init(int pwm_p, int tach_p, float setpoint);
  void get_setpoint(float setpoint);
  void tach_isr();
  void update(float dt_sec);
};

#endif // H_WHEEL
