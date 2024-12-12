/*
 * Arquivo de protótipo de funçõe do publisher.
 */

#ifndef PUBLISHER_H
#define PUBLISHER_H

#include <Arduino.h>
#include <UltrasonicSensor.h>
#include <MPU6050_tockn.h>
#include <sensor_msgs/msg/range.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/float64_multi_array.h>
#include "config_topics.h"
#include "config_pins.h"
#include "magnetic_encoder.h"
#include "utils.h"
#include "rosidl_runtime_c/string_functions.h"

extern "C" int clock_gettime(clockid_t unused, struct timespec * tp);   /* Função para contagem de tempo em segundo e nanosegundo */

void publish_message(
  pub_topic_config_t * topic_config, 
  sensor_config_t * sensor_config
);                                    /* Publica dados dos sensores do robô com base na configuração do tópico */
void publish_message_type_Imu(
  pub_topic_config_t * topic_config, 
  sensor_config_t * sensor_config
);                                    /* Publica mensagem do tipo IMU (sensor_msgs/msg) */
void publish_message_type_Range(
  pub_topic_config_t * topic_config, 
  sensor_config_t * sensor_config
);                                    /* Publica mensagem do tipo Range (sensor_msgs/msg) */
void publish_message_type_Float64MultiArray(
  pub_topic_config_t * topic_config, 
  sensor_config_t * sensor_config
);                                    /* Publica mensagem do tipo Float32MultiArray (std_msgs/msg) */

#endif /* PUBLISHER_H */