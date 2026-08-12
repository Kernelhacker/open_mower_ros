//
// Created by clemens on 25.07.24.
//

#ifndef MOWERSERVICEINTERFACE_H
#define MOWERSERVICEINTERFACE_H

#include <mower_msgs/Status.h>
#include <ros/publisher.h>
#include <sensor_msgs/Range.h>

#include <MowerServiceInterfaceBase.hpp>

class MowerServiceInterface : public MowerServiceInterfaceBase {
 public:
  MowerServiceInterface(uint16_t service_id, const xbot::serviceif::Context& ctx,
                        const ros::Publisher& status_publisher, const ros::Publisher& us_left_publisher,
                        const ros::Publisher& us_right_publisher)
      : MowerServiceInterfaceBase(service_id, ctx),
        status_publisher_(status_publisher),
        us_left_publisher_(us_left_publisher),
        us_right_publisher_(us_right_publisher) {
  }

  void SetMowerSpeed(float speed);

  void Tick();

 protected:
  void OnMowerStatusChanged(const uint8_t& new_value) override;
  void OnRainDetectedChanged(const uint8_t& new_value) override;
  void OnMowerRunningChanged(const uint8_t& new_value) override;
  void OnMowerESCTemperatureChanged(const float& new_value) override;
  void OnMowerMotorTemperatureChanged(const float& new_value) override;
  void OnMowerMotorCurrentChanged(const float& new_value) override;
  void OnMowerMotorRPMChanged(const float& new_value) override;
  void OnUltrasonicLeftChanged(const float& new_value) override;
  void OnUltrasonicRightChanged(const float& new_value) override;

 private:
  void OnServiceConnected(uint16_t service_id) override;
  void OnTransactionStart(uint64_t timestamp) override;
  void OnTransactionEnd() override;

 private:
  mower_msgs::Status status_msg_{};
  const ros::Publisher& status_publisher_;
  const ros::Publisher& us_left_publisher_;
  const ros::Publisher& us_right_publisher_;
  float commanded_speed_ = 0.0f;  // commanded normalized speed in [-1, 1]; re-sent every tick
};

#endif  // MOWERSERVICEINTERFACE_H
