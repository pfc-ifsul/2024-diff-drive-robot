/*
 * Arquivo de protótipo de funções das tarefas do FreeRTOS.
 *
 * Este arquivo define quais são as tarefas que irão realizar as publicações dos sensores, 
 *  as inscrições nos tópicos de controle do robô e o controle PI.
 */

#ifndef FREERTOS_TASKS_H
#define FREERTOS_TASKS_H

#include <Arduino.h>
#include <UltrasonicSensor.h>
#include <MPU6050_tockn.h>
#include <Wire.h>
#include <std_msgs/msg/float64_multi_array.h>
#include "config_topics.h"
#include "publisher.h"
#include "config_pins.h"
#include "magnetic_encoder.h"
#include "utils.h"
#include "controller.h"

void IRAM_ATTR right_wheel_encoder(void);              /* Interrupção para contagem de pulsos do encoder da roda direita */
void IRAM_ATTR left_wheel_encoder(void);               /* Interrupção para contagem de pulsos do encoder da roda esquerda */

void subscription_callback(const void * msgin);        /* Função de callback da tarefa "sub_WheelSpeedControl" */

void task_pub_IMU(void * pvParameters);                /* Tarefa responsável pela publicação do sensor IMU */
void task_pub_UltrasonicMiddle(void * pvParameters);   /* Tarefa responsável pela publicação do sensor ultrassônico do centro */
void task_pub_UltrasonicLeft(void * pvParameters);     /* Tarefa responsável pela publicação do sensor ultrassônico da esquerda */
void task_pub_UltrasonicRight(void * pvParameters);    /* Tarefa responsável pela publicação do sensor ultrassônico da direita */
void task_pub_WheelSpeed(void * pvParameters);         /* Tarefa responsável pela publicação da velocidade das rodas */
void task_sub_WheelSpeedControl(void * pvParameters);  /* Tarefa responsável pela inscrição no tópico de velocidade desejada para as rodas */  
void task_WheelSpeedControl(void * pvParameters);      /* Tarefa responsável pelo controle de velocidade das rodas */  

#endif /* FREERTOS_TASKS_H */