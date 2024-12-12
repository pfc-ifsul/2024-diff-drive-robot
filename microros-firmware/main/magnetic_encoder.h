/*
 * Arquivo de definição do sensor magnético.
 *
 * Este arquivo define o sensor encoder magnético.
 */

#ifndef MAGNETIC_ENCODER_H
#define MAGNETIC_ENCODER_H

#include <Arduino.h>
#include "config_topics.h"

#define ENCODER_TICKS_PER_REVOLUTION 616
#define RPM_TO_RAD_PER_SEC 0.1047197551

class MagneticEncoder {                         /* Encoder magnético */
public:
  MagneticEncoder();                            
  void incrementRightWheelPulseCount(void);     /* Incrementa a variável "right_wheel_pulse_count" a cada borda de subida do sensor */
  void incrementLeftWheelPulseCount(void);      /* Incrementa a variável "left_wheel_pulse_count" a cada borda de subida do sensor */
  float getRightWheelVelocityRad(void);         /* Obtém a velocidade da roda direita em rad/s */
  float getLeftWheelVelocityRad(void);          /* Obtém a velocidade da roda esquerda em rad/s */
  void updateRightWheelVelocity(void);          /* Calcula a velocdiade da roda direita */
  void updateLeftWheelVelocity(void);           /* Calcula a velocdiade da roda esquerda */
  ~MagneticEncoder();

private:
  void resetRightWheelCount(void);              /* Reseta o valor da variável "right_wheel_pulse_count" */
  void resetLeftWheelCount(void);               /* Reseta o valor da variável "left_wheel_pulse_count" */
  uint16_t getRightWheelPulseCount(void);       /* Obtém o valor da variável "right_wheel_pulse_count" */
  uint16_t getLeftWheelPulseCount(void);        /* Obtém o valor da variável "left_wheel_pulse_count" */

  volatile uint16_t right_wheel_pulse_count;
  volatile uint16_t left_wheel_pulse_count;
  uint16_t right_wheel_pulse_count_copy;
  uint16_t left_wheel_pulse_count_copy;
  float right_wheel_rpm;
  float left_wheel_rpm;
};

#endif /* MAGNETIC_ENCODER_H */