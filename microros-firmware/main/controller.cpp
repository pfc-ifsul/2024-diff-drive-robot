#include "controller.h"

Controller::Controller(float Kp, float Ki) 
  : Kp(Kp),
    Ki(Ki),
    integral_error(0.0),
    current_time(0.0),
    previous_time(0.0) {}

void Controller::setup_pins(uint8_t MOTOR_IN1, uint8_t MOTOR_IN2, uint8_t MOTOR_EN, uint8_t PWM_CHANNEL) {
  this->MOTOR_IN1 = MOTOR_IN1;
  this->MOTOR_IN2 = MOTOR_IN2;
  this->MOTOR_EN = MOTOR_EN;
  this->PWM_CHANNEL = PWM_CHANNEL;

  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_EN, OUTPUT);
  ledcAttachChannel(
    MOTOR_EN,  
    PWM_FREQUENCY, 
    PWM_RESOLUTION,
    PWM_CHANNEL
  );

  stop_wheel();
}

float Controller::updateTime(void) {
  current_time = xTaskGetTickCount(); 
  float dt = (float)(current_time - previous_time) / 1000.0;
  previous_time = current_time;

  return dt;
}

float Controller::getError(float setpoint_speed, float current_speed) {
  return setpoint_speed - current_speed;
}

int Controller::compute(float setpoint_speed, float current_speed) {
  float dt = updateTime();
  float error = getError(setpoint_speed, current_speed);

  integral_error += error * dt;
  integral_error = constrain(integral_error, MIN_VALUE_INTEGRAL, MAX_VALUE_INTEGRAL);

  float u = Kp* error + Ki * integral_error;

  return (int)constrain(u, MIN_VALUE_PI_OUTPUT, MAX_VALUE_PI_OUTPUT);
}

void Controller::set_wheel_speed(int duty_cycle) {
  ledcWrite(MOTOR_EN, duty_cycle);
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);
}

void Controller::stop_wheel(void) {
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
}

Controller::~Controller() {}
