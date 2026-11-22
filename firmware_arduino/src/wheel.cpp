#include <Arduino.h>
#include "config.hpp"
#include "wheel.hpp"

void Wheel::init(int pwm_p, int tach_p, float setpoint)
{
    pwm_pin = pwm_p;
    tach_pin = tach_p;
    target_velocity = setpoint;

    pinMode(pwm_pin, OUTPUT);
    digitalWrite(pwm_pin, LOW);
    pinMode(tach_pin, INPUT);
}

void Wheel::get_setpoint(float setpoint)
{
    target_velocity = setpoint;
}

void Wheel::tach_isr()
{
    unsigned long now = micros();
    if (last_pulse_us != 0)
    {
        unsigned long dt = now - last_pulse_us;

        if (dt < MIN_PULSE_PERIOD_US)
        {
            return;
        }

        if (dt > STOP_TIMEOUT_US)
        {
            last_pulse_us = now;
            return;
        }

        delta_us = dt;
        last_pulse_us = now;
    }
    else
    {
        last_pulse_us = now;
    }
}

void Wheel::update(float dt_sec)
{
    unsigned long dt;
    unsigned long last;
    unsigned long now_us;

    noInterrupts();
    dt = delta_us;
    last = last_pulse_us;
    now_us = micros();
    interrupts();

    if (last == 0 || (now_us - last) > STOP_TIMEOUT_US)
    {
        current_velocity = 0.0f;
    }
    else
    {
        unsigned long elapsed = now_us - last;
        unsigned long effective_dt = dt;

        if (elapsed > effective_dt)
        {
            effective_dt = elapsed;
        }

        if (effective_dt > 0)
        {
            current_velocity = ANGLE_PER_PULSE / (effective_dt * 1e-6f);
        }
        else
        {
            current_velocity = 0.0f;
        }
    }

    if (target_velocity <= 0.001f)
    {
        integral_error = 0.0f;
        control_output = 0;
        analogWrite(pwm_pin, 0);
        return;
    }

    float error = target_velocity - current_velocity;

    integral_error += error * dt_sec;
    integral_error = constrain(integral_error, -MAX_INTEGRAL, MAX_INTEGRAL);

    float u = MIN_PWM_DEADZONE + (K_FF * target_velocity) + (KP * error) + (KI * integral_error);

    if (u > MAX_PWM)
    {
        control_output = MAX_PWM;
        if (error > 0.0f)
        {
            integral_error -= error * dt_sec;
        }
    }
    else if (u < 0.0f)
    {
        control_output = 0;
        if (error < 0.0f)
        {
            integral_error -= error * dt_sec;
        }
    }
    else
    {
        control_output = (int)u;
    }

    analogWrite(pwm_pin, control_output);
}
