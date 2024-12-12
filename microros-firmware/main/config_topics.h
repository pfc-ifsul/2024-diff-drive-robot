/*
 * Arquivo de configuração de frequências de publicação.
 *
 * Este arquivo define a frequência de publicação utilizada pelos sensores.
 * Este arquivo define o nome dos tópicos de publicação e inscrição.
 */

#ifndef CONFIG_TOPICS_H
#define CONFIG_TOPICS_H

#include <stdint.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#define FREQ_TO_TICK(freq) ( (TickType_t)( 1000 / (freq * portTICK_PERIOD_MS) ) )

#define PUB_WHEEL_SPEED_CONTROL_ARRAY_CAPACITY    2     /* Quantos elementos o array de publicação, que envia a velocidade das rodas, vai receber */
#define SUB_WHEEL_SPEED_CONTROL_ARRAY_CAPACITY    2     /* Quantos elementos o array de inscrição, que recebe a velocidade de controle das rodas, vai receber */
#define ULTRASONIC_FREQ                           2     /* Frequência de publicação dos ultrassônicos em Hz */
#define IMU_FREQ                                  20    /* Frequência de publicação do IMU em Hz */
#define ENCODER_FREQ                              20    /* Frequência de publicação das velocidades das rodas em Hz */

const char * const PUB_TOPIC_NAME_IMU = "/robot/sensor/imu";                              /* Publicação do sensor IMU */
const char * const PUB_TOPIC_NAME_ULTRASONIC_MIDDLE = "/robot/sensor/ultrasonic/center";  /* Publicação do sensor ultrassônico central */
const char * const PUB_TOPIC_NAME_ULTRASONIC_LEFT = "/robot/sensor/ultrasonic/left";      /* Publicação do sensor ultrassônico esquerdo */
const char * const PUB_TOPIC_NAME_ULTRASONIC_RIGHT = "/robot/sensor/ultrasonic/right";    /* Publicação do sensor ultrassônico direito */
const char * const PUB_TOPIC_NAME_WHEEL_SPEED_ENCODER = "/robot/sensor/wheel/encoder";    /* Publicação da velocidade das rodas */
const char * const SUB_TOPIC_NAME_WHEEL_CONTROL_SPEED = "/robot/control/wheel/speed";     /* Inscrição do controle de velocidade das rodas */

typedef enum {
  SENSOR_MSGS_IMU,
  SENSOR_MSGS_RANGE,
  STD_MSGS_FLOAT64MULTIARRAY
} message_type_t;                   /* Tipo de menssagem que será publicada pela tarefa */

typedef struct {
  rcl_node_t node;
  rcl_allocator_t allocator;
  rclc_support_t support;
  rclc_executor_t executor;
} node_config_t;                   /* Estrutura para configurar o nó criado pelo micro-ros */

typedef struct {
  const char * topic_name;
  uint16_t publish_freq;
  rcl_publisher_t publisher;
  void * message;
  message_type_t message_type;
} pub_topic_config_t;             /* Estrutura para configurar os tópicos de publicação */

typedef struct {
  const char * topic_name;
  rcl_subscription_t subscriber;
  void * message;
  message_type_t message_type;
  node_config_t * node_config;
} sub_topic_config_t;             /* Estrutura para configurar os tópicos de inscrição */

#endif /* CONFIG_TOPICS_H */