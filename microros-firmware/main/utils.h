/*
 * Arquivo com funções auxiliares.
 *
 * Este arquivo define funções auxiliares que são utilizadas entre as tarefas do FreeRTOS.
 */

#ifndef UTILS_H
#define UTILS_H

#include <freertos/FreeRTOS.h>
#include <Arduino.h>
#include "magnetic_encoder.h"

#define GRAVITY         9.80665
#define DEG_TO_RAD      0.01745329252            

/* Variáveis para controlar o acesso à memória */
extern SemaphoreHandle_t get_wheel_speed_mutex;         /* Mutex definido no main.ino */
extern SemaphoreHandle_t desired_wheel_speed_mutex;     /* Mutex definido no main.ino */
extern float desired_right_wheel_speed;                 /* Variável de velocidade desejada definida no main.ino */
extern float desired_left_wheel_speed;                  /* Variável de velocidade desejada definida no main.ino */

typedef struct {
  float x;
  float y;
  float z;
  float w;
} quaternion_t;                 /* Tipo de dado, quaternion */

quaternion_t euler_to_quaternion(
  float roll,
  float pitch,
  float yaw);                   /* Converte ângulos de euler em quaternion. OBS: os parâmetros são em graus */

void get_wheels_speed(
  MagneticEncoder * encoder, 
  float &right_wheel, 
  float &left_wheel);           /* Função que gerencia o acesso à leitura da velocidade das rodas */

void get_desired_wheels_speed(
  float &right_wheel, 
  float &left_wheel);           /* Função que gerencia o acesso à leitura da velocidade desejada das rodas */

void set_desired_wheels_speed(
  float right_wheel,
  float left_wheel);            /* Função que gerencia o acesso à escrita da velocidade desejada das rodas */

#endif /* UTILS_H */