//
// Created by Clemens Elflein on 15.03.22.
// Copyright (c) 2022 Clemens Elflein and OpenMower contributors. All rights reserved.
//
// This file is part of OpenMower.
//
// OpenMower is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
// License as published by the Free Software Foundation, version 3 of the License.
//
// OpenMower is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
// warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along with OpenMower. If not, see
// <https://www.gnu.org/licenses/>.
//
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mower_msgs/ESCStatus.h>
#include <mower_msgs/Emergency.h>
#include <mower_msgs/EmergencyStopSrv.h>
#include <mower_msgs/HighLevelControlSrv.h>
#include <mower_msgs/MowerControlSrv.h>
#include <nmea_msgs/Sentence.h>
#include <ros/ros.h>
#include <rtcm_msgs/Message.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <spdlog/sinks/callback_sink.h>
#include <spdlog/spdlog.h>
#include <std_msgs/String.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include "../../../services/service_ids.h"
#include "BmsServiceInterface.h"
#include "DiffDriveServiceInterface.h"
#include "EmergencyServiceInterface.h"
#include "GpsServiceInterface.h"
#include "HighLevelServiceInterface.h"
#include "ImuServiceInterface.h"
#include "InputServiceInterface.h"
#include "MowerServiceInterface.h"
#include "PowerServiceInterface.h"

ros::Publisher status_pub;
ros::Publisher us_left_pub;
ros::Publisher us_right_pub;
ros::Publisher nmea_pub;
ros::Publisher power_pub;
ros::Publisher bms_pub;
ros::Publisher gps_position_pub;
ros::Publisher status_left_esc_pub;
ros::Publisher status_right_esc_pub;
ros::Publisher emergency_pub;
ros::Publisher actual_twist_pub;
ros::Publisher action_pub;

ros::Publisher sensor_imu_pub;

ros::ServiceClient highLevelClient;

std::unique_ptr<EmergencyServiceInterface> emergency_service = nullptr;
std::unique_ptr<DiffDriveServiceInterface> diff_drive_service = nullptr;
std::unique_ptr<MowerServiceInterface> mower_service = nullptr;
std::unique_ptr<ImuServiceInterface> imu_service = nullptr;
std::unique_ptr<PowerServiceInterface> power_service = nullptr;
std::unique_ptr<BmsServiceInterface> bms_service = nullptr;
std::unique_ptr<GpsServiceInterface> gps_service = nullptr;
std::unique_ptr<InputServiceInterface> input_service = nullptr;
std::unique_ptr<HighLevelServiceInterface> high_level_service = nullptr;

xbot::serviceif::Context ctx{};

bool setEmergencyStop(mower_msgs::EmergencyStopSrvRequest& req, mower_msgs::EmergencyStopSrvResponse& res) {
  emergency_service->SetHighLevelEmergency(req.reason);
  return true;
}

void velReceived(const geometry_msgs::Twist::ConstPtr& msg) {
  diff_drive_service->SendTwist(msg);
}

void rtcmReceived(const rtcm_msgs::Message& msg) {
  static std::vector<uint8_t> rtcm_buffer{};
  static ros::Time last_time_sent{0};
  ros::Time now = ros::Time::now();
  // Append the bytes to the buffer
  rtcm_buffer.insert(rtcm_buffer.end(), msg.message.begin(), msg.message.end());
  // In order to not spam after each received byte, limit packets to 5Hz and to max 1k of size
  if (rtcm_buffer.size() < 1000 && (now - last_time_sent).toSec() < 0.2) return;
  last_time_sent = now;
  gps_service->SendRTCM(rtcm_buffer.data(), rtcm_buffer.size());
  rtcm_buffer.clear();
}

void sendEmergencyHeartbeatTimerTask(const ros::TimerEvent&) {
  emergency_service->Heartbeat();
}

void actionReceived(const std_msgs::String::ConstPtr& action) {
  input_service->OnAction(action->data);
}

void sendMowerEnabledTimerTask(const ros::TimerEvent& e) {
  mower_service->Tick();
}

bool setMowEnabled(mower_msgs::MowerControlSrvRequest& req, mower_msgs::MowerControlSrvResponse& res) {
  // enable + direction -> signed speed: +1 forward, -1 reverse, 0 off.
  const float speed = req.mow_enabled ? (req.mow_direction ? 1.0f : -1.0f) : 0.0f;
  mower_service->SetMowerSpeed(speed);
  return true;
}

static void spdlog_cb(const spdlog::details::log_msg& msg) {
  ros::console::Level level = ros::console::Level::Info;
  switch (msg.level) {
    case spdlog::level::level_enum::trace:
    case spdlog::level::level_enum::debug: level = ros::console::Level::Debug; break;
    case spdlog::level::level_enum::info: break;
    case spdlog::level::level_enum::warn: level = ros::console::Level::Warn; break;
    case spdlog::level::level_enum::err: level = ros::console::Level::Error; break;
    case spdlog::level::level_enum::critical: level = ros::console::Level::Fatal; break;
    case spdlog::level::level_enum::off: return;
  }
  ROS_LOG(level, ROSCONSOLE_DEFAULT_NAME, "%.*s", static_cast<int>(msg.payload.size()), msg.payload.data());
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "mower_comms_v2");

  {
    auto sink = std::make_shared<spdlog::sinks::callback_sink_mt>(spdlog_cb);
    auto logger = std::make_shared<spdlog::logger>("", std::move(sink));
    spdlog::set_default_logger(logger);
  }

  ros::NodeHandle n;
  ros::NodeHandle paramNh("/ll");

  highLevelClient = n.serviceClient<mower_msgs::HighLevelControlSrv>("mower_service/high_level_control");
  action_pub = n.advertise<std_msgs::String>("xbot/action", 1);

  std::string bind_ip = "0.0.0.0";
  paramNh.getParam("bind_ip", bind_ip);
  ROS_INFO_STREAM("Bind IP (Robot Internal): " << bind_ip);
  xbot::serviceif::SetShutdownCallback([] { ros::requestShutdown(); });
  ctx = xbot::serviceif::Start(true, bind_ip);

  // Emergency service
  emergency_pub = n.advertise<mower_msgs::Emergency>("ll/emergency", 1);
  emergency_service = std::make_unique<EmergencyServiceInterface>(xbot::service_ids::EMERGENCY, ctx, emergency_pub);
  emergency_service->Start();

  // Diff drive service
  actual_twist_pub = n.advertise<geometry_msgs::TwistStamped>("ll/diff_drive/measured_twist", 1);
  status_left_esc_pub = n.advertise<mower_msgs::ESCStatus>("ll/diff_drive/left_esc_status", 1);
  status_right_esc_pub = n.advertise<mower_msgs::ESCStatus>("ll/diff_drive/right_esc_status", 1);
  double wheel_ticks_per_m = 0.0;
  double wheel_distance_m = 0.0;
  if (!paramNh.getParam("services/diff_drive/ticks_per_m", wheel_ticks_per_m)) {
    ROS_ERROR("Need to provide param services/diff_drive/ticks_per_m");
    return 1;
  }
  if (!paramNh.getParam("services/diff_drive/wheel_distance_m", wheel_distance_m)) {
    ROS_ERROR("Need to provide param services/diff_drive/wheel_distance_m");
    return 1;
  }
  ROS_INFO_STREAM("Wheel ticks [1/m]: " << wheel_ticks_per_m);
  ROS_INFO_STREAM("Wheel distance [m]: " << wheel_distance_m);

  int baud_rate = 0;
  paramNh.getParam("services/gps/baud_rate", baud_rate);

  std::string protocol;
  paramNh.getParam("services/gps/protocol", protocol);

  int gps_port_index = 0;
  paramNh.getParam("services/gps/port_index", gps_port_index);

  if (baud_rate == 0 || protocol.empty()) {
    ROS_ERROR("Need to specify GPS protocol and baud rate!");
    return 1;
  }

  ROS_INFO_STREAM("GPS protocol: " << protocol << ", baud rate: " << baud_rate
                                   << ", gps port index:" << gps_port_index);

  diff_drive_service = std::make_unique<DiffDriveServiceInterface>(xbot::service_ids::DIFF_DRIVE, ctx, actual_twist_pub,
                                                                   status_left_esc_pub, status_right_esc_pub,
                                                                   wheel_ticks_per_m, wheel_distance_m);
  diff_drive_service->Start();

  // Mower service
  status_pub = n.advertise<mower_msgs::Status>("ll/mower_status", 1);
  us_left_pub = n.advertise<sensor_msgs::Range>("ll/ultrasonic/left", 1);
  us_right_pub = n.advertise<sensor_msgs::Range>("ll/ultrasonic/right", 1);
  mower_service =
      std::make_unique<MowerServiceInterface>(xbot::service_ids::MOWER, ctx, status_pub, us_left_pub, us_right_pub);
  mower_service->Start();

  // Static transforms for ultrasonic sensors (configurable via custom_params.yaml)
  static tf2_ros::StaticTransformBroadcaster static_broadcaster;
  std::vector<geometry_msgs::TransformStamped> static_transforms;

  double us_left_x = 0.18, us_left_y = 0.105, us_left_z = 0.1;
  double us_left_yaw = 0.0, us_left_pitch = 0.0, us_left_roll = 0.0;
  paramNh.getParam("ultrasonic/left/x", us_left_x);
  paramNh.getParam("ultrasonic/left/y", us_left_y);
  paramNh.getParam("ultrasonic/left/z", us_left_z);
  paramNh.getParam("ultrasonic/left/yaw", us_left_yaw);
  paramNh.getParam("ultrasonic/left/pitch", us_left_pitch);
  paramNh.getParam("ultrasonic/left/roll", us_left_roll);

  geometry_msgs::TransformStamped tf_left;
  tf_left.header.stamp = ros::Time::now();
  tf_left.header.frame_id = "base_link";
  tf_left.child_frame_id = "ultrasonic_left_link";
  tf_left.transform.translation.x = us_left_x;
  tf_left.transform.translation.y = us_left_y;
  tf_left.transform.translation.z = us_left_z;
  tf2::Quaternion q_left;
  q_left.setRPY(us_left_roll, us_left_pitch, us_left_yaw);
  tf_left.transform.rotation.x = q_left.x();
  tf_left.transform.rotation.y = q_left.y();
  tf_left.transform.rotation.z = q_left.z();
  tf_left.transform.rotation.w = q_left.w();
  static_transforms.push_back(tf_left);

  double us_right_x = 0.18, us_right_y = -0.105, us_right_z = 0.1;
  double us_right_yaw = 0.0, us_right_pitch = 0.0, us_right_roll = 0.0;
  paramNh.getParam("ultrasonic/right/x", us_right_x);
  paramNh.getParam("ultrasonic/right/y", us_right_y);
  paramNh.getParam("ultrasonic/right/z", us_right_z);
  paramNh.getParam("ultrasonic/right/yaw", us_right_yaw);
  paramNh.getParam("ultrasonic/right/pitch", us_right_pitch);
  paramNh.getParam("ultrasonic/right/roll", us_right_roll);

  geometry_msgs::TransformStamped tf_right;
  tf_right.header.stamp = ros::Time::now();
  tf_right.header.frame_id = "base_link";
  tf_right.child_frame_id = "ultrasonic_right_link";
  tf_right.transform.translation.x = us_right_x;
  tf_right.transform.translation.y = us_right_y;
  tf_right.transform.translation.z = us_right_z;
  tf2::Quaternion q_right;
  q_right.setRPY(us_right_roll, us_right_pitch, us_right_yaw);
  tf_right.transform.rotation.x = q_right.x();
  tf_right.transform.rotation.y = q_right.y();
  tf_right.transform.rotation.z = q_right.z();
  tf_right.transform.rotation.w = q_right.w();
  static_transforms.push_back(tf_right);

  static_broadcaster.sendTransform(static_transforms);

  // IMU service
  std::string imu_axis_config;
  paramNh.getParam("services/imu/axis_config", imu_axis_config);
  ROS_INFO_STREAM("IMU axis config: " << imu_axis_config);
  sensor_imu_pub = n.advertise<sensor_msgs::Imu>("ll/imu/data_raw", 1);
  imu_service = std::make_unique<ImuServiceInterface>(xbot::service_ids::IMU, ctx, sensor_imu_pub, imu_axis_config);
  imu_service->Start();

  // Power service
  power_pub = n.advertise<mower_msgs::Power>("ll/power", 1);

  // Mainly for monitoring and informational purposes
  float battery_full_voltage;
  float battery_empty_voltage;
  float battery_critical_voltage;
  float battery_critical_high_voltage;
  if (!paramNh.getParam("services/power/battery_full_voltage", battery_full_voltage)) {
    ROS_ERROR("Need to set param: services/power/battery_full_voltage");
    return 1;
  }
  if (!paramNh.getParam("services/power/battery_empty_voltage", battery_empty_voltage)) {
    ROS_ERROR("Need to set param: services/power/battery_empty_voltage");
    return 1;
  }
  if (!paramNh.getParam("services/power/battery_critical_voltage", battery_critical_voltage)) {
    ROS_ERROR("Need to set param: services/power/battery_critical_voltage");
    return 1;
  }
  if (!paramNh.getParam("services/power/battery_critical_high_voltage", battery_critical_high_voltage)) {
    ROS_ERROR("Need to set param: services/power/battery_critical_high_voltage");
    return 1;
  }

  // Optional charger configuration
  float charge_voltage = -1.0f;
  float charge_current = -1.0f;
  float charge_termination_current = -1.0f;
  float charge_precharge_current = -1.0f;
  int charge_recharge_voltage = -1;
  paramNh.getParam("services/power/charge_voltage", charge_voltage);
  paramNh.getParam("services/power/charge_current", charge_current);
  paramNh.getParam("services/power/charge_termination_current", charge_termination_current);
  paramNh.getParam("services/power/charge_pre_charge_current", charge_precharge_current);
  paramNh.getParam("services/power/charge_re_charge_voltage", charge_recharge_voltage);

  // Optional settings also required for charger DPM (dynamic power management)
  float system_current = -1.0f;  // Max. current allowed to be drawn from wall AC/DC
  paramNh.getParam("services/power/system_current", system_current);
  bool override_hw_charge_current_limit = false;
  paramNh.getParam("services/power/dangerously_override_hardware_charge_current_limit",
                   override_hw_charge_current_limit);
  power_service = std::make_unique<PowerServiceInterface>(
      xbot::service_ids::POWER, ctx, power_pub, battery_full_voltage, battery_empty_voltage, battery_critical_voltage,
      battery_critical_high_voltage, charge_voltage, charge_current, charge_termination_current,
      charge_precharge_current, charge_recharge_voltage, system_current, override_hw_charge_current_limit);
  power_service->Start();

  // BMS service
  bms_pub = n.advertise<mower_msgs::Bms>("ll/bms", 1);
  bms_service = std::make_unique<BmsServiceInterface>(xbot::service_ids::BMS, ctx, bms_pub);
  bms_service->Start();

  // GPS service
  double datum_lat, datum_long, datum_height;
  bool has_datum = true;
  has_datum &= paramNh.getParam("services/gps/datum_lat", datum_lat);
  has_datum &= paramNh.getParam("services/gps/datum_long", datum_long);
  has_datum &= paramNh.getParam("services/gps/datum_height", datum_height);
  if (!has_datum) {
    ROS_ERROR_STREAM("You need to provide datum_lat and datum_long and datum_height in order to use the absolute mode");
    return 2;
  }
  ROS_INFO_STREAM("Datum: " << datum_lat << ", " << datum_long << ", " << datum_height);
  gps_position_pub = n.advertise<xbot_msgs::AbsolutePose>("ll/position/gps", 1);
  nmea_pub = n.advertise<nmea_msgs::Sentence>("ll/position/gps/nmea", 1);
  bool absolute_coords = true;
  paramNh.getParam("services/gps/absolute_coords", absolute_coords);
  gps_service = std::make_unique<GpsServiceInterface>(xbot::service_ids::GPS, ctx, gps_position_pub, nmea_pub,
                                                      datum_lat, datum_long, datum_height, baud_rate, protocol,
                                                      gps_port_index, absolute_coords);
  gps_service->Start();

  // Input service
  {
    std::string config_file = paramNh.param<std::string>("services/input/config_file", "");
    int lift_multiple_delay = paramNh.param("services/input/lift_multiple_delay", -1);
    int collision_multiple_delay = paramNh.param("services/input/collision_multiple_delay", -1);
    input_service = std::make_unique<InputServiceInterface>(xbot::service_ids::INPUT, ctx, config_file,
                                                            lift_multiple_delay, collision_multiple_delay, action_pub);
    input_service->Start();
  }

  // HighLevel service
  high_level_service = std::make_unique<HighLevelServiceInterface>(xbot::service_ids::HIGH_LEVEL, ctx);
  high_level_service->Start();

  // All subscriptions, timers and service servers are registered after all service interfaces are
  // fully constructed, so callbacks can never fire on null pointers.
  ros::ServiceServer mow_service = n.advertiseService("ll/_service/mow_enabled", setMowEnabled);
  ros::ServiceServer ros_emergency_service = n.advertiseService("ll/_service/emergency", setEmergencyStop);
  ros::Subscriber cmd_vel_sub = n.subscribe("ll/cmd_vel", 0, velReceived, ros::TransportHints().tcpNoDelay(true));
  ros::Subscriber rtcm_sub = n.subscribe("ll/position/gps/rtcm", 0, rtcmReceived);
  // ros::Subscriber high_level_status_sub = n.subscribe("/mower_logic/current_state", 0, highLevelStatusReceived);
  ros::Timer publish_timer = n.createTimer(ros::Duration(0.5), sendEmergencyHeartbeatTimerTask);
  ros::Timer publish_timer_2 = n.createTimer(ros::Duration(5.0), sendMowerEnabledTimerTask);
  ros::Subscriber action_sub = n.subscribe("xbot/action", 0, actionReceived, ros::TransportHints().tcpNoDelay(true));

  ROS_INFO("All mower_comms_v2 services started");

  ros::spin();
  xbot::serviceif::Stop();

  return 0;
}
