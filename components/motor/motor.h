#ifndef MOTOR_H
#define MOTOR_H
#include "esp_log.h"
#include <driver/uart.h>


// Define os pinos do motor
#define PWM_PIN 5
#define IN1_PIN 15
#define IN2_PIN 27  // Substituição para o pino 13 no ESP32

#define BUF_SIZE (1024)

void setup();

void motor_uart_config(int baud_rate, int tx_pin, int rx_pin);
void attachInterrupt();

// Funções para configurar e controlar o motor
void setupMotor();
void setMotor(int dir, int pwmVal);

#endif