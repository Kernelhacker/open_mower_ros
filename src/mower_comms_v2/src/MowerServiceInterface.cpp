//
// Created by clemens on 25.07.24.
//

#include "MowerServiceInterface.h"

#include <sensor_msgs/Range.h>

void MowerServiceInterface::Tick() {
  SendMowerSpeed(commanded_speed_);
}

void MowerServiceInterface::SetMowerSpeed(float speed) {
  // Signed speed/duty in [-1, 1]: sign = direction, 0 = off.
  commanded_speed_ = speed;
  SendMowerSpeed(speed);
  status_msg_.mow_enabled = speed != 0.0f;
  status_publisher_.publish(status_msg_);
}

void MowerServiceInterface::OnMowerStatusChanged(const uint8_t& new_value) {
  status_msg_.mower_status = new_value;
}

void MowerServiceInterface::OnRainDetectedChanged(const uint8_t& new_value) {
  status_msg_.rain_detected = new_value;
}

void MowerServiceInterface::OnMowerRunningChanged(const uint8_t& new_value) {
  // TODO: set a flag, if the mower is actually running or not.
}

void MowerServiceInterface::OnMowerESCTemperatureChanged(const float& new_value) {
  status_msg_.mower_esc_temperature = new_value;
}

void MowerServiceInterface::OnMowerMotorTemperatureChanged(const float& new_value) {
  status_msg_.mower_motor_temperature = new_value;
}

void MowerServiceInterface::OnMowerMotorCurrentChanged(const float& new_value) {
  status_msg_.mower_esc_current = new_value;
}

void MowerServiceInterface::OnMowerMotorRPMChanged(const float& new_value) {
  status_msg_.mower_motor_rpm = new_value;
}

void MowerServiceInterface::OnUltrasonicLeftChanged(const float& new_value) {
  us_left_range_ = new_value;
  has_us_left_ = true;
}

void MowerServiceInterface::OnUltrasonicRightChanged(const float& new_value) {
  us_right_range_ = new_value;
  has_us_right_ = true;
}

void MowerServiceInterface::OnServiceConnected(uint16_t service_id) {
  status_msg_ = {};
  has_us_left_ = false;
  has_us_right_ = false;
  // Clear the cached speed so a stale value isn't replayed after reconnect.
  commanded_speed_ = 0.0f;
  SendMowerSpeed(commanded_speed_);
}

void MowerServiceInterface::OnTransactionStart(uint64_t timestamp) {
  status_msg_.stamp = ros::Time::now();
}

void MowerServiceInterface::OnTransactionEnd() {
  status_publisher_.publish(status_msg_);

  ros::Time now = ros::Time::now();
  if (has_us_left_) {
    sensor_msgs::Range msg;
    msg.header.stamp = now;
    msg.header.frame_id = "ultrasonic_left_link";
    msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
    msg.field_of_view = 0.523;  // ~30 degrees
    msg.min_range = 0.1;
    msg.max_range = 2.0;
    msg.range = us_left_range_;
    us_left_publisher_.publish(msg);
  }
  if (has_us_right_) {
    sensor_msgs::Range msg;
    msg.header.stamp = now;
    msg.header.frame_id = "ultrasonic_right_link";
    msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
    msg.field_of_view = 0.523;  // ~30 degrees
    msg.min_range = 0.1;
    msg.max_range = 2.0;
    msg.range = us_right_range_;
    us_right_publisher_.publish(msg);
  }
}
