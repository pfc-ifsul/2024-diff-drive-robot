/*
 * Arquivo com funções auxiliares.
 *
 * Este arquivo define funções auxiliares que são utilizadas entre as tarefas do FreeRTOS.
 */

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <Arduino.h>
#include "config_pins.h"

#define PI_CONTROL_FREQ 50                            /* Frequência de atualização do controle PI */
#define MAX_VALUE_PI_OUTPUT 2047
#define MIN_VALUE_PI_OUTPUT 0 
#define MAX_VALUE_INTEGRAL 5
#define MIN_VALUE_INTEGRAL -5 

const float KP = 800.0;
const float KI = 1000.0;
const uint32_t PWM_FREQUENCY = 500;
const uint8_t PWM_RESOLUTION = 11;
const uint8_t PWM_CHANNEL_RIGHT = 0;
const uint8_t PWM_CHANNEL_LEFT = 1;

class Controller {
public:
  Controller(float, float); 
  void setup_pins(uint8_t, uint8_t, uint8_t, uint8_t);
  void stop_wheel(void);
  void set_wheel_speed(int);
  int compute(float, float);
  ~Controller();

private:
  float updateTime(void);
  float getError(float, float);

  uint8_t MOTOR_IN1; 
  uint8_t MOTOR_IN2; 
  uint8_t MOTOR_EN; 
  uint8_t PWM_CHANNEL;
  float Kp;
  float Ki;
  float integral_error;
  TickType_t current_time;
  TickType_t previous_time;
};

#endif /* CONTROLLER_H */