#include "config_freertos.h"
#include "config_topics.h"

#include <sensor_msgs/msg/range.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/float64_multi_array.h>

/*
 * Tarefas responsáveis pelas publicações e inscrições dos tópicos.
 */
extern void task_pub_IMU(void * pvParameters);
extern void task_pub_UltrasonicMiddle(void * pvParameters);
extern void task_pub_UltrasonicLeft(void * pvParameters);
extern void task_pub_UltrasonicRight(void * pvParameters);
extern void task_pub_WheelSpeed(void * pvParameters);
extern void task_sub_WheelSpeedControl(void * pvParameters);
extern void task_WheelSpeedControl(void * pvParameters);

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
node_config_t node_config;

/*
 * Variáveis de controle de memória
 */
SemaphoreHandle_t get_wheel_speed_mutex;        /* Controle de acesso aos endereços de leitura de velocidade das rodas */
SemaphoreHandle_t desired_wheel_speed_mutex;    /* Controle de acesso aos endereço de leitura de velocidade desejada para as rodas */

/*
 * Variáveis para setar a velocidade desejada para as rodas inicialmente
 */
float desired_right_wheel_speed = 0.0;
float desired_left_wheel_speed = 0.0;

/* 
 * Configuração dos tópicos de publicação
 */
rcl_publisher_t imu_publisher;
sensor_msgs__msg__Imu imu_msg;
pub_topic_config_t imu_topic_config = {
  .topic_name = PUB_TOPIC_NAME_IMU,
  .publish_freq = IMU_FREQ,
  .publisher = imu_publisher,
  .message = &imu_msg,
  .message_type = SENSOR_MSGS_IMU
};

rcl_publisher_t ultrasonic_middle_publisher;
sensor_msgs__msg__Range ultrasonic_middle_msg;
pub_topic_config_t ultrasonic_middle_topic_config = {
  .topic_name = PUB_TOPIC_NAME_ULTRASONIC_MIDDLE,
  .publish_freq = ULTRASONIC_FREQ,
  .publisher = ultrasonic_middle_publisher,
  .message = &ultrasonic_middle_msg,
  .message_type = SENSOR_MSGS_RANGE
};

rcl_publisher_t ultrasonic_left_publisher;
sensor_msgs__msg__Range ultrasonic_left_msg;
pub_topic_config_t ultrasonic_left_topic_config = {
  .topic_name = PUB_TOPIC_NAME_ULTRASONIC_LEFT,
  .publish_freq = ULTRASONIC_FREQ,
  .publisher = ultrasonic_left_publisher,
  .message = &ultrasonic_left_msg,
  .message_type = SENSOR_MSGS_RANGE
};

rcl_publisher_t ultrasonic_right_publisher;
sensor_msgs__msg__Range ultrasonic_right_msg;
pub_topic_config_t ultrasonic_right_topic_config = {
  .topic_name = PUB_TOPIC_NAME_ULTRASONIC_RIGHT,
  .publish_freq = ULTRASONIC_FREQ,
  .publisher = ultrasonic_right_publisher,
  .message = &ultrasonic_right_msg,
  .message_type = SENSOR_MSGS_RANGE
};

rcl_publisher_t wheel_speed_publisher;
std_msgs__msg__Float64MultiArray wheel_speed_pub_msg;
pub_topic_config_t wheel_speed_topic_pub_config = {
  .topic_name = PUB_TOPIC_NAME_WHEEL_SPEED_ENCODER,
  .publish_freq = ENCODER_FREQ,
  .publisher = wheel_speed_publisher,
  .message = &wheel_speed_pub_msg,
  .message_type = STD_MSGS_FLOAT64MULTIARRAY
};

/* 
 * Configuração dos tópicos de inscrição
 */
rcl_subscription_t wheel_speed_subscriber;
std_msgs__msg__Float64MultiArray wheel_speed_sub_msg;
sub_topic_config_t wheel_speed_topic_sub_config = {
  .topic_name = SUB_TOPIC_NAME_WHEEL_CONTROL_SPEED,
  .subscriber = wheel_speed_subscriber,
  .message = &wheel_speed_sub_msg,
  .message_type = STD_MSGS_FLOAT64MULTIARRAY,
  .node_config = &node_config
};

void setup() {
  Serial.begin(115200);
  set_microros_wifi_transports("OsirMax_Casa988", "1975CvbV", "192.168.18.69", 8888);
  //set_microros_wifi_transports("MI-9", "esp32-ifsul", "192.168.255.113", 8888);
  delay(2000);

  /* Inicialização dos mutex */
  get_wheel_speed_mutex = xSemaphoreCreateMutex();
  desired_wheel_speed_mutex = xSemaphoreCreateMutex();

  /* Node */
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "micro_ros_arduino_wifi_node", "", &support);

  /* Inicialização do node_config */
  node_config.node = node;
  node_config.support = support;
  node_config.allocator = allocator;
  node_config.executor = executor;

  /* Executor */
  rclc_executor_init(&node_config.executor, &node_config.support.context, 1, &node_config.allocator);

  /* Definindo a quantidade de memória necessária para o array que recebe a velocidade de controle das rodas */
  wheel_speed_sub_msg.data.capacity = SUB_WHEEL_SPEED_CONTROL_ARRAY_CAPACITY;
  wheel_speed_sub_msg.data.size = 0;
  wheel_speed_sub_msg.data.data = (double *) malloc(wheel_speed_sub_msg.data.capacity * sizeof(double));
  wheel_speed_sub_msg.layout.dim.capacity = SUB_WHEEL_SPEED_CONTROL_ARRAY_CAPACITY;
  wheel_speed_sub_msg.layout.dim.size = 0;
  wheel_speed_sub_msg.layout.dim.data = (std_msgs__msg__MultiArrayDimension*) malloc(wheel_speed_sub_msg.layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension));
  /* Definindo a quantidade de memória necessária para o array que publica a velocidade das rodas */
  wheel_speed_pub_msg.data.capacity = PUB_WHEEL_SPEED_CONTROL_ARRAY_CAPACITY;
  wheel_speed_pub_msg.data.size = 0;
  wheel_speed_pub_msg.data.data = (double *) malloc(wheel_speed_pub_msg.data.capacity * sizeof(double));
  wheel_speed_pub_msg.layout.dim.capacity = PUB_WHEEL_SPEED_CONTROL_ARRAY_CAPACITY;
  wheel_speed_pub_msg.layout.dim.size = 0;
  wheel_speed_pub_msg.layout.dim.data = (std_msgs__msg__MultiArrayDimension*) malloc(wheel_speed_pub_msg.layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension));


  /* Publishers */
  rclc_publisher_init_best_effort(
    &imu_topic_config.publisher,
    &node_config.node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    imu_topic_config.topic_name);

  rclc_publisher_init_best_effort(
    &ultrasonic_middle_topic_config.publisher,
    &node_config.node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
    ultrasonic_middle_topic_config.topic_name);

  rclc_publisher_init_best_effort(
    &ultrasonic_left_topic_config.publisher,
    &node_config.node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
    ultrasonic_left_topic_config.topic_name);

  rclc_publisher_init_best_effort(
    &ultrasonic_right_topic_config.publisher,
    &node_config.node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
    ultrasonic_right_topic_config.topic_name);

  rclc_publisher_init_best_effort(
    &wheel_speed_topic_pub_config.publisher,
    &node_config.node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
    wheel_speed_topic_pub_config.topic_name);

  /* Subscribers */
  rclc_subscription_init_default(
    &wheel_speed_topic_sub_config.subscriber,
    &node_config.node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
    wheel_speed_topic_sub_config.topic_name);

  xTaskCreatePinnedToCore(
    task_pub_IMU, 
    "Publish_Imu", 
    TASK_MEMORY_SIZE_4KB, 
    (void *)&imu_topic_config, 
    TASK_PRIORITY_ONE, 
    NULL, 
    TASK_CORE_1);

  xTaskCreatePinnedToCore(
    task_pub_UltrasonicMiddle, 
    "Publish_Ultrasonic_Middle", 
    TASK_MEMORY_SIZE_4KB, 
    (void *)&ultrasonic_middle_topic_config, 
    TASK_PRIORITY_ONE, 
    NULL, 
    TASK_CORE_0);

  xTaskCreatePinnedToCore(
    task_pub_UltrasonicLeft, 
    "Publish_Ultrasonic_Left", 
    TASK_MEMORY_SIZE_4KB, 
    (void *)&ultrasonic_left_topic_config, 
    TASK_PRIORITY_ONE, 
    NULL, 
    TASK_CORE_0);

  xTaskCreatePinnedToCore(
    task_pub_UltrasonicRight, 
    "Publish_Ultrasonic_Right", 
    TASK_MEMORY_SIZE_4KB, 
    (void *)&ultrasonic_right_topic_config, 
    TASK_PRIORITY_ONE, 
    NULL, 
    TASK_CORE_0);

  xTaskCreatePinnedToCore(
    task_pub_WheelSpeed, 
    "Publish_Wheel_Speed", 
    TASK_MEMORY_SIZE_4KB, 
    (void *)&wheel_speed_topic_pub_config, 
    TASK_PRIORITY_ONE, 
    NULL, 
    TASK_CORE_1);

  xTaskCreatePinnedToCore(
    task_sub_WheelSpeedControl, 
    "Subscribe_Wheel_Desired_Speed", 
    TASK_MEMORY_SIZE_4KB, 
    (void *)&wheel_speed_topic_sub_config, 
    TASK_PRIORITY_ONE, 
    NULL, 
    TASK_CORE_1);

  xTaskCreatePinnedToCore(
    task_WheelSpeedControl, 
    "Wheel_Speed_Control", 
    TASK_MEMORY_SIZE_4KB, 
    NULL, 
    TASK_PRIORITY_ONE, 
    NULL, 
    TASK_CORE_1);
}

void loop() {}
