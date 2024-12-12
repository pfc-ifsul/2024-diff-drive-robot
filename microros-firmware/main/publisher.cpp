#include "publisher.h"

void publish_message(pub_topic_config_t * topic_config, sensor_config_t * sensor_config) {
  switch (topic_config->message_type) {
    case SENSOR_MSGS_IMU:
      publish_message_type_Imu(topic_config, sensor_config);
      break;
    case SENSOR_MSGS_RANGE:
      publish_message_type_Range(topic_config, sensor_config);
      break;
    case STD_MSGS_FLOAT64MULTIARRAY:
      publish_message_type_Float64MultiArray(topic_config, sensor_config);
      break;
    default:
      Serial.println("Topic message type invalid");
      break;
  }
}

void publish_message_type_Imu(pub_topic_config_t * topic_config, sensor_config_t * sensor_config) {
  sensor_msgs__msg__Imu * msg = (sensor_msgs__msg__Imu *)topic_config->message;
  MPU6050 * sensor = (MPU6050 *)sensor_config->sensor_class_constructor;
  struct timespec ts;
  quaternion_t q;
  q = euler_to_quaternion(sensor->getAngleX(), sensor->getAngleY(), sensor->getAngleZ());
  clock_gettime(CLOCK_REALTIME, &ts);
  sensor->update();
  msg->header.stamp.sec = ts.tv_sec;
  msg->header.stamp.nanosec = ts.tv_nsec;
  rosidl_runtime_c__String__assign(&msg->header.frame_id, "base_footprint");
  msg->orientation.x = q.x;
  msg->orientation.y = q.y;
  msg->orientation.z = q.z;
  msg->orientation.w = q.w;
  msg->orientation_covariance[0] = 0.0000001; 
  msg->orientation_covariance[4] = 0.0000001;
  msg->orientation_covariance[8] = 0.0000001; 
  msg->orientation_covariance[1] = msg->orientation_covariance[2] = 0.0; 
  msg->orientation_covariance[3] = msg->orientation_covariance[5] = 0.0;
  msg->orientation_covariance[6] = msg->orientation_covariance[7] = 0.0;
  msg->angular_velocity.x = sensor->getGyroX() * DEG_TO_RAD;
  msg->angular_velocity.y = sensor->getGyroY() * DEG_TO_RAD;
  msg->angular_velocity.z = sensor->getGyroZ() * DEG_TO_RAD;
  msg->angular_velocity_covariance[0] = 0.008;
  msg->angular_velocity_covariance[4] = 0.008;
  msg->angular_velocity_covariance[8] = 0.005; 
  msg->angular_velocity_covariance[1] = msg->angular_velocity_covariance[2] = 0.0;
  msg->angular_velocity_covariance[3] = msg->angular_velocity_covariance[5] = 0.0;
  msg->angular_velocity_covariance[6] = msg->angular_velocity_covariance[7] = 0.0;
  msg->linear_acceleration.x = sensor->getAccX() * GRAVITY;
  msg->linear_acceleration.y = sensor->getAccY() * GRAVITY;
  msg->linear_acceleration.z = sensor->getAccZ() * GRAVITY;
  msg->linear_acceleration_covariance[0] = 1.1; 
  msg->linear_acceleration_covariance[4] = 0.02; 
  msg->linear_acceleration_covariance[8] = 0.000001; 
  msg->linear_acceleration_covariance[1] = msg->linear_acceleration_covariance[2] = 0.0;
  msg->linear_acceleration_covariance[3] = msg->linear_acceleration_covariance[5] = 0.0;
  msg->linear_acceleration_covariance[6] = msg->linear_acceleration_covariance[7] = 0.0;
  rcl_publish(&topic_config->publisher, msg, NULL);
}

void publish_message_type_Range(pub_topic_config_t * topic_config, sensor_config_t * sensor_config) {
  sensor_msgs__msg__Range * msg = (sensor_msgs__msg__Range *)topic_config->message;
  UltrasonicSensor * sensor = (UltrasonicSensor *)sensor_config->sensor_class_constructor;
  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  msg->header.stamp.sec = ts.tv_sec;
  msg->header.stamp.nanosec = ts.tv_nsec;
  msg->radiation_type = 0;
  msg->range = (float) sensor->distanceInCentimeters();
  msg->min_range = 1.0;
  msg->max_range = 400.0;
  msg->field_of_view = 21.0;
  rcl_publish(&topic_config->publisher, msg, NULL);
}

void publish_message_type_Float64MultiArray(pub_topic_config_t * topic_config, sensor_config_t * sensor_config) {
  std_msgs__msg__Float64MultiArray * msg = (std_msgs__msg__Float64MultiArray *)topic_config->message;
  MagneticEncoder * sensor = (MagneticEncoder *)sensor_config->sensor_class_constructor;
  float right_wheel_speed, left_wheel_speed;
  sensor->updateRightWheelVelocity();
  sensor->updateLeftWheelVelocity();
  get_wheels_speed(sensor, right_wheel_speed, left_wheel_speed);
  msg->data.data[0] = right_wheel_speed;
  msg->data.data[1] = left_wheel_speed;
  msg->data.size = 2;
  rcl_publish(&topic_config->publisher, msg, NULL);
}
