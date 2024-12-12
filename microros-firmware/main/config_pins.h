/*
 * Arquivo de configuração de pinos do projeto.
 *
 * Este arquivo define os pinos utilizados no projeto para os diversos sensores e atuadores.
 */

#ifndef CONFIG_PINS_H
#define CONFIG_PINS_H

#include <stdint.h>

#define ECHO_PIN_MIDDLE         15 /* Pino de ECHO do sensor ultrassônico localizado na parte frontal do robô, meio */
#define TRIGGER_PIN_MIDDLE      2  /* Pino de TRIGGER do sensor ultrassônico localizado na parte frontal do robô, meio */
#define ECHO_PIN_LEFT           18 /* Pino de ECHO do sensor ultrassônico localizado na parte frontal do robô, esquerda */
#define TRIGGER_PIN_LEFT        19 /* Pino de TRIGGER do sensor ultrassônico localizado na parte frontal do robô, esquerda */
#define ECHO_PIN_RIGHT          4  /* Pino de ECHO do sensor ultrassônico localizado na parte frontal do robô, direita */
#define TRIGGER_PIN_RIGHT       5  /* Pino de TRIGGER do sensor ultrassônico localizado na parte frontal do robô, direita */
#define SDA_PIN                 21 /* Pino SDA do sensor IMU */
#define SCL_PIN                 22 /* Pino SCL do sensor IMU */
#define MOTOR_RIGHT_IN3         14 /* Entrada IN1 de controle do atuador, motor direito */
#define MOTOR_RIGHT_IN4         27 /* Entrada IN2 de controle do atuador, motor direito */
#define MOTOR_RIGHT_ENABLE      13 /* Entrada ENA que habilita o atuador, motor direito */
#define MOTOR_RIGHT_ENCODER     26 /* Entrada de pulsos do encoder, motor direito */
#define MOTOR_LEFT_IN1          32 /* Entrada IN1 de controle do atuador, motor esquerdo */
#define MOTOR_LEFT_IN2          33 /* Entrada IN2 de controle do atuador, motor esquerdo */
#define MOTOR_LEFT_ENABLE       25 /* Entrada ENB que habilita o atuador, motor esquerdo */
#define MOTOR_LEFT_ENCODER      23 /* Entrada de pulsos do encoder, motor esquerdo */

typedef enum {
  SENSOR_HC_SR04,
  SENSOR_MPU6050,
  SENSOR_MAGNETIC_ENCODER
} sensor_type_t;              /* Tipos de sensores utilizados no robô */

typedef struct {
  uint8_t echoPin;
  uint8_t triggerPin;
} ultrasonic_pin_config_t;    /* Estrutura para configurar os pinos do sensor ultrassônico */

typedef struct {
  uint8_t sclPin;
  uint8_t sdaPin;
} imu_pin_config_t;           /* Estrutura para configurar os pinos do sensor imu */

typedef struct {
  uint8_t encoderA;
  uint8_t encoderB;
} encoder_pin_config_t;       /* Estrutura para configurar os pinos do sensor encoder */

typedef struct {
  void * sensor_pin;
  void * sensor_class_constructor;
  sensor_type_t sensor_type;
} sensor_config_t;             /* Estrutura para configurar os sensores */

#endif /* CONFIG_PINS_H */