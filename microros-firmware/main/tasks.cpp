#include "tasks.h"

void task_pub_IMU(void * pvParameters) {
  pub_topic_config_t * topic_config = (pub_topic_config_t *)pvParameters;
  Serial.println(topic_config->topic_name);
  Serial.println(topic_config->message_type);
  imu_pin_config_t imu_pin_config = { 
    .sclPin = SCL_PIN, 
    .sdaPin = SDA_PIN 
  };
  MPU6050 mpu6050(Wire);
  Wire.begin(imu_pin_config.sdaPin, imu_pin_config.sclPin);
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  sensor_config_t sensor_config = {
    .sensor_pin = &imu_pin_config,
    .sensor_class_constructor = &mpu6050,
    .sensor_type = SENSOR_MPU6050
  };
  for (;;) {
    publish_message(topic_config, &sensor_config);
    vTaskDelay(FREQ_TO_TICK(topic_config->publish_freq));
  }
}

void task_pub_UltrasonicMiddle(void * pvParameters) {
  pub_topic_config_t * topic_config = (pub_topic_config_t *)pvParameters;
  Serial.println(topic_config->topic_name);
  Serial.println(topic_config->message_type);
  ultrasonic_pin_config_t ultrasonic_pin_config = {
    .echoPin = ECHO_PIN_MIDDLE,
    .triggerPin = TRIGGER_PIN_MIDDLE
  };
  UltrasonicSensor ultrasonic(ultrasonic_pin_config.triggerPin, ultrasonic_pin_config.echoPin);
  sensor_config_t sensor_middle_config = {
    .sensor_pin = &ultrasonic_pin_config,
    .sensor_class_constructor = &ultrasonic,
    .sensor_type = SENSOR_HC_SR04
  };
  for (;;) {
    publish_message(topic_config, &sensor_middle_config);
    vTaskDelay(FREQ_TO_TICK(topic_config->publish_freq));
  }
}

void task_pub_UltrasonicLeft(void * pvParameters) {
  pub_topic_config_t * topic_config = (pub_topic_config_t *)pvParameters;
  Serial.println(topic_config->topic_name);
  Serial.println(topic_config->message_type);
  ultrasonic_pin_config_t ultrasonic_pin_config = {
    .echoPin = ECHO_PIN_LEFT,
    .triggerPin = TRIGGER_PIN_LEFT
  };
  UltrasonicSensor ultrasonic(ultrasonic_pin_config.triggerPin, ultrasonic_pin_config.echoPin);
  sensor_config_t sensor_middle_config = {
    .sensor_pin = &ultrasonic_pin_config,
    .sensor_class_constructor = &ultrasonic,
    .sensor_type = SENSOR_HC_SR04
  };
  for (;;) {
    publish_message(topic_config, &sensor_middle_config);
    vTaskDelay(FREQ_TO_TICK(topic_config->publish_freq));
  }
}

void task_pub_UltrasonicRight(void * pvParameters) {
  pub_topic_config_t * topic_config = (pub_topic_config_t *)pvParameters;
  Serial.println(topic_config->topic_name);
  Serial.println(topic_config->message_type);
  ultrasonic_pin_config_t ultrasonic_pin_config = {
    .echoPin = ECHO_PIN_RIGHT,
    .triggerPin = TRIGGER_PIN_RIGHT
  };
  UltrasonicSensor ultrasonic(ultrasonic_pin_config.triggerPin, ultrasonic_pin_config.echoPin);
  sensor_config_t sensor_middle_config = {
    .sensor_pin = &ultrasonic_pin_config,
    .sensor_class_constructor = &ultrasonic,
    .sensor_type = SENSOR_HC_SR04
  };
  for (;;) {
    publish_message(topic_config, &sensor_middle_config);
    vTaskDelay(FREQ_TO_TICK(topic_config->publish_freq));
  }
}

MagneticEncoder encoder;

void right_wheel_encoder() {
  encoder.incrementRightWheelPulseCount();
}

void left_wheel_encoder() {
  encoder.incrementLeftWheelPulseCount();
}

void task_pub_WheelSpeed(void * pvParameters) {
  pub_topic_config_t * topic_config = (pub_topic_config_t *)pvParameters;
  Serial.println(topic_config->topic_name);
  Serial.println(topic_config->message_type);
  encoder_pin_config_t encoder_pin_config = {
    .encoderA = MOTOR_RIGHT_ENCODER,
    .encoderB = MOTOR_LEFT_ENCODER
  };
  encoder = MagneticEncoder();
  sensor_config_t sensor_encoder_config = {
    .sensor_pin = &encoder_pin_config,
    .sensor_class_constructor = &encoder,
    .sensor_type = SENSOR_MAGNETIC_ENCODER
  };
  attachInterrupt(digitalPinToInterrupt(encoder_pin_config.encoderA), right_wheel_encoder, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder_pin_config.encoderB), left_wheel_encoder, RISING);
  for (;;) {
    publish_message(topic_config, &sensor_encoder_config);
    vTaskDelay(FREQ_TO_TICK(topic_config->publish_freq));
  }
}

void callback_task_sub_WheelSpeedControl(const void * msgin) {
  const std_msgs__msg__Float64MultiArray * msg = (const std_msgs__msg__Float64MultiArray *)msgin;
  if (msg->data.size == 2) {
    set_desired_wheels_speed(msg->data.data[0], msg->data.data[1]);
  }
}

void task_sub_WheelSpeedControl(void * pvParameters) {
  sub_topic_config_t * topic_config = (sub_topic_config_t *)pvParameters;
  Serial.println(topic_config->topic_name);
  Serial.println(topic_config->message_type);
  rclc_executor_add_subscription(
    &topic_config->node_config->executor, 
    &topic_config->subscriber, 
    (std_msgs__msg__Float64MultiArray *)topic_config->message, 
    &callback_task_sub_WheelSpeedControl, 
    ON_NEW_DATA);
  for(;;) {
    rclc_executor_spin_some(&topic_config->node_config->executor, RCL_MS_TO_NS(100));
  }
}   

void task_WheelSpeedControl(void * pvParameters) {
  Serial.println("PI controller task");
  float right_wheel_speed, left_wheel_speed;
  float right_wheel_setpoint, left_wheel_setpoint;
  int right_wheel_duty_cycle;
  int left_wheel_duty_cycle;
  Controller right_wheel_controller(KP, KI);
  Controller left_wheel_controller(KP, KI);
  right_wheel_controller.setup_pins(MOTOR_RIGHT_IN3, MOTOR_RIGHT_IN4, MOTOR_RIGHT_ENABLE, PWM_CHANNEL_RIGHT);
  left_wheel_controller.setup_pins(MOTOR_LEFT_IN2, MOTOR_LEFT_IN1, MOTOR_LEFT_ENABLE, PWM_CHANNEL_LEFT);
  for(;;) {
    get_wheels_speed(&encoder, right_wheel_speed, left_wheel_speed);
    get_desired_wheels_speed(right_wheel_setpoint, left_wheel_setpoint);
    right_wheel_duty_cycle = right_wheel_controller.compute(right_wheel_setpoint, right_wheel_speed);
    left_wheel_duty_cycle = left_wheel_controller.compute(left_wheel_setpoint, left_wheel_speed);
    right_wheel_controller.set_wheel_speed(right_wheel_duty_cycle);
    left_wheel_controller.set_wheel_speed(left_wheel_duty_cycle);
    vTaskDelay(FREQ_TO_TICK(PI_CONTROL_FREQ));
  }
}
