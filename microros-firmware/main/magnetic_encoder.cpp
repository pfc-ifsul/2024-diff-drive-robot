#include "magnetic_encoder.h"

MagneticEncoder::MagneticEncoder() 
  : right_wheel_pulse_count(0),
    left_wheel_pulse_count(0),
    right_wheel_rpm(0),
    left_wheel_rpm(0) {}

void MagneticEncoder::incrementRightWheelPulseCount(void) {
  right_wheel_pulse_count++;
}

void MagneticEncoder::incrementLeftWheelPulseCount(void) {
  left_wheel_pulse_count++;
}

uint16_t MagneticEncoder::getRightWheelPulseCount(void) {
  return right_wheel_pulse_count;
} 

uint16_t MagneticEncoder::getLeftWheelPulseCount(void) {
  return left_wheel_pulse_count;
} 

void MagneticEncoder::resetRightWheelCount(void) {
  right_wheel_pulse_count = 0;
}

void MagneticEncoder::resetLeftWheelCount(void) {
  left_wheel_pulse_count = 0;
}

void MagneticEncoder::updateRightWheelVelocity(void) {
  noInterrupts();
  right_wheel_pulse_count_copy = getRightWheelPulseCount();
  resetRightWheelCount();
  interrupts();

  right_wheel_rpm = (float)((right_wheel_pulse_count_copy * 60.0 / ENCODER_TICKS_PER_REVOLUTION) * ENCODER_FREQ);
}

void MagneticEncoder::updateLeftWheelVelocity(void) {
  noInterrupts();
  left_wheel_pulse_count_copy = getLeftWheelPulseCount();
  resetLeftWheelCount();
  interrupts();

  left_wheel_rpm = (float)((left_wheel_pulse_count_copy * 60.0 / ENCODER_TICKS_PER_REVOLUTION) * ENCODER_FREQ);
}

float MagneticEncoder::getRightWheelVelocityRad(void) {
  return right_wheel_rpm * RPM_TO_RAD_PER_SEC;
}

float MagneticEncoder::getLeftWheelVelocityRad(void) {
  return left_wheel_rpm * RPM_TO_RAD_PER_SEC;
}

MagneticEncoder::~MagneticEncoder() {}
