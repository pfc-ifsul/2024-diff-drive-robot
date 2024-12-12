#include "utils.h"

extern SemaphoreHandle_t get_wheel_speed_mutex;
extern SemaphoreHandle_t desired_wheel_speed_mutex;
extern float desired_right_wheel_speed;
extern float desired_left_wheel_speed;

void get_wheels_speed(MagneticEncoder * encoder, float &right_wheel, float &left_wheel) {
  if (xSemaphoreTake(get_wheel_speed_mutex, portMAX_DELAY) == pdTRUE) {
    right_wheel = encoder->getRightWheelVelocityRad();
    left_wheel = encoder->getLeftWheelVelocityRad();
    xSemaphoreGive(get_wheel_speed_mutex);
  }
}

void get_desired_wheels_speed(float &right_wheel, float &left_wheel) {
  if (xSemaphoreTake(desired_wheel_speed_mutex, portMAX_DELAY) == pdTRUE) {
    right_wheel = desired_right_wheel_speed;
    left_wheel = desired_left_wheel_speed;
    xSemaphoreGive(desired_wheel_speed_mutex);
  }
}

void set_desired_wheels_speed(float right_wheel, float left_wheel) {
  if (xSemaphoreTake(desired_wheel_speed_mutex, portMAX_DELAY) == pdTRUE) {
    desired_right_wheel_speed = right_wheel;
    desired_left_wheel_speed = left_wheel;
    xSemaphoreGive(desired_wheel_speed_mutex);
  }
}

quaternion_t euler_to_quaternion(float roll, float pitch, float yaw) {
  float cr = cos(roll * DEG_TO_RAD * 0.5);
  float sr = sin(roll * DEG_TO_RAD * 0.5);
  float cp = cos(pitch * DEG_TO_RAD * 0.5);
  float sp = sin(pitch * DEG_TO_RAD * 0.5);
  float cy = cos(yaw * DEG_TO_RAD * 0.5);
  float sy = sin(yaw * DEG_TO_RAD * 0.5);

  quaternion_t q;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;
  q.w = cr * cp * cy + sr * sp * sy;
  
  return q;
}
