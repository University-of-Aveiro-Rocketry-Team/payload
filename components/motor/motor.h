#ifndef MOTOR_H
#define MOTOR_H
#include "esp_log.h"
#include <driver/uart.h>
#include <driver/ledc.h>

// Define os pinos do motor
#define ENCA 2      // Encoder A (input)
#define ENCB 4      // Encoder B (input)
#define PWM 5       // Motor driver PWM (ESP32 GPIO) (output)
#define IN1 15      // Motor driver IN1 (output)
#define IN2 27      // Motor driver IN2 (output)

#define BUF_SIZE (1024)

#define MOTOR_RESOLUTION LEDC_TIMER_8_BIT
#define MOTOR_FREQ 1000
#define MOTOR_CHANNEL LEDC_CHANNEL_0

void motor_setup();

void attachInterrupt();
void setup_motor_pwm(int PWM_PIN, int PWM_FREQ, ledc_timer_bit_t PWM_RES, ledc_channel_t PWM_CHANNEL);
void ledc_write(float duty_cycle);

// Funções para configurar e controlar o motor
void loop_motor();
void setMotor(int dir, int pwmVal, int in1, int in2);
void readEncoder();

#endif