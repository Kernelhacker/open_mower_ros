// Created by Clemens Elflein on 2/21/22.
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
#include "MowingBehavior.h"

#include <cryptopp/cryptlib.h>
#include <cryptopp/hex.h>
#include <cryptopp/sha.h>
#include <geometry_msgs/Polygon.h>
#include <mbf_msgs/RecoveryAction.h>
#include <nav_msgs/Path.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Empty.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <EmergencyServiceInterfaceBase.hpp>
#include <cmath>
#include <limits>

#include "../StateSubscriber.h"
#include "mower_logic/CheckPoint.h"
#include "mower_map/ClearNavPointSrv.h"
#include "mower_map/GetMowingAreaSrv.h"
#include "mower_map/SetNavPointSrv.h"
#include "mower_msgs/Emergency.h"
#include "mower_msgs/Status.h"
#include "xbot_msgs/AbsolutePose.h"

extern ros::ServiceClient mapClient;
extern ros::ServiceClient pathClient;
extern ros::ServiceClient pathProgressClient;
extern ros::ServiceClient setNavPointClient;
extern ros::ServiceClient clearNavPointClient;
extern ros::Publisher add_dynamic_obstacle_pub;
extern ros::Publisher clear_dynamic_obstacles_pub;
extern StateSubscriber<sensor_msgs::Range> us_left_state_subscriber;
extern StateSubscriber<sensor_msgs::Range> us_right_state_subscriber;

extern xbot_msgs::AbsolutePose getPose();

extern actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction>* mbfClient;
extern actionlib::SimpleActionClient<mbf_msgs::ExePathAction>* mbfClientExePath;
extern actionlib::SimpleActionClient<mbf_msgs::RecoveryAction>* mbfClientRecovery;
extern mower_logic::MowerLogicConfig getConfig();
extern void setConfig(mower_logic::MowerLogicConfig);

extern void registerActions(std::string prefix, const std::vector<xbot_msgs::ActionInfo>& actions);
extern void setEmergencyMode(uint16_t reason);

extern StateSubscriber<mower_msgs::Status> status_state_subscriber;
extern StateSubscriber<mower_msgs::Emergency> emergency_state_subscriber;

extern std::string current_job_id;
extern bool current_job_finished;

namespace {
bool isPointInPolygon(double x, double y, const std::vector<geometry_msgs::Point32>& poly) {
  bool inside = false;
  size_t n = poly.size();
  for (size_t i = 0, j = n - 1; i < n; j = i++) {
    double xi = poly[i].x, yi = poly[i].y;
    double xj = poly[j].x, yj = poly[j].y;
    bool intersect = ((yi > y) != (yj > y)) && (x < (xj - xi) * (y - yi) / (yj - yi) + xi);
    if (intersect) inside = !inside;
  }
  return inside;
}
}  // namespace

MowingBehavior MowingBehavior::INSTANCE;

std::string MowingBehavior::state_name() {
  if (paused) {
    return "PAUSED";
  }
  return "MOWING";
}

Behavior* MowingBehavior::execute() {
  shared_state->active_semiautomatic_task = true;

  while (ros::ok() && !aborted) {
    if (currentMowingPaths.empty() && !create_mowing_plan(currentMowingArea)) {
      ROS_INFO_STREAM("MowingBehavior: Could not create mowing plan, docking");
      // Start again from first area next time.
      reset();
      // We cannot create a plan, so we're probably done. Go to docking station
      return &DockingBehavior::INSTANCE;
    }

    // No plan will be created if the area is skipped
    if (currentMowingPaths.empty()) {
      currentMowingArea++;
      currentMowingPath = 0;
      currentMowingPathIndex = 0;
      continue;
    }

    // We have a plan, execute it
    ROS_INFO_STREAM("MowingBehavior: Executing mowing plan");
    bool finished = execute_mowing_plan();
    if (finished) {
      // skip to next area if current
      ROS_INFO_STREAM("MowingBehavior: Executing mowing plan - finished");
      currentMowingArea++;
      currentMowingPaths.clear();
      currentMowingPath = 0;
      currentMowingPathIndex = 0;
    }
  }

  if (!ros::ok()) {
    // something went wrong
    return nullptr;
  }
  // we got aborted, go to docking station
  return &DockingBehavior::INSTANCE;
}

void MowingBehavior::enter() {
  skip_area = false;
  skip_path = false;
  paused = aborted = false;

  for (auto& a : actions) {
    a.enabled = true;
  }
  registerActions("mower_logic:mowing", actions);
}

void MowingBehavior::exit() {
  for (auto& a : actions) {
    a.enabled = false;
  }
  registerActions("mower_logic:mowing", actions);
}

void MowingBehavior::reset() {
  publishMowerEvent("JOB_COMPLETE");
  publishMowerEvent("TEMPORARY_OBSTACLES_CLEARED", json{{"obstacles", json::array()}});
  clear_dynamic_obstacles_pub.publish(std_msgs::Empty());
  publishMqtt("temporary_obstacles/json", json::array(), true);
  current_job_finished = true;
  currentMowingPaths.clear();
  temporary_obstacles.clear();
  currentMowingArea = 0;
  currentMowingPath = 0;
  currentMowingPathIndex = 0;
  // increase cumulative mowing angle offset increment
  currentMowingAngleIncrementSum = std::fmod(currentMowingAngleIncrementSum + getConfig().mow_angle_increment, 360);
  checkpoint();

  if (config.automatic_mode == eAutoMode::SEMIAUTO) {
    ROS_INFO_STREAM("MowingBehavior: Finished semiautomatic task");
    shared_state->active_semiautomatic_task = false;
  }
}

bool MowingBehavior::needs_gps() {
  return true;
}

bool MowingBehavior::mower_enabled() {
  return mowerEnabled;
}

void MowingBehavior::update_actions() {
  for (auto& a : actions) {
    a.enabled = true;
  }

  // pause / resume switch. other actions are always available
  actions[0].enabled = !(requested_pause_flag & pauseType::PAUSE_MANUAL);
  actions[1].enabled = requested_pause_flag & pauseType::PAUSE_MANUAL;

  registerActions("mower_logic:mowing", actions);
}

bool MowingBehavior::create_mowing_plan(int area_index) {
  ROS_INFO_STREAM("MowingBehavior: Creating mowing plan for area: " << area_index);
  // Delete old plan and progress.
  currentMowingPaths.clear();

  // get the mowing area
  mower_map::GetMowingAreaSrv mapSrv;
  mapSrv.request.index = area_index;
  if (!mapClient.call(mapSrv)) {
    ROS_ERROR_STREAM("MowingBehavior: Error loading mowing area");
    return false;
  }

  currentMowingAreaId = mapSrv.response.area.id;
  currentMowingAreaName = mapSrv.response.area.name;

  if (!mapSrv.response.area.active) {
    ROS_INFO_STREAM("MowingBehavior: Skipping inactive mowing area");
    return true;
  }

  // Area orientation is the same as the first point, unless explicitly specified in the area attributes
  double angle = 0;
  if (!std::isnan(mapSrv.response.area.angle)) {
    angle = mapSrv.response.area.angle;
    ROS_INFO_STREAM("MowingBehavior: Using explicitly specified mow angle: " << angle);
  } else {
    auto points = mapSrv.response.area.area.points;
    if (points.size() >= 2) {
      tf2::Vector3 first(points[0].x, points[0].y, 0);
      for (auto point : points) {
        tf2::Vector3 second(point.x, point.y, 0);
        auto diff = second - first;
        if (diff.length() > 2.0) {
          // we have found a point that has a distance of > 2 m, calculate the angle
          angle = atan2(diff.y(), diff.x());
          ROS_INFO_STREAM("MowingBehavior: Detected mow angle: " << angle);
          break;
        }
      }
    }
  }

  // add mowing angle offset increment and return into the <-180, 180> range
  double mow_angle_offset = std::fmod(getConfig().mow_angle_offset + currentMowingAngleIncrementSum + 180, 360);
  if (mow_angle_offset < 0) mow_angle_offset += 360;
  mow_angle_offset -= 180;
  ROS_INFO_STREAM("MowingBehavior: mowing angle offset (deg): " << mow_angle_offset);
  if (config.mow_angle_offset_is_absolute) {
    angle = mow_angle_offset * (M_PI / 180.0);
    ROS_INFO_STREAM("MowingBehavior: Custom mowing angle: " << angle);
  } else {
    angle = angle + mow_angle_offset * (M_PI / 180.0);
    ROS_INFO_STREAM("MowingBehavior: Auto-detected mowing angle + mowing angle offset: " << angle);
  }

  // calculate coverage
  const auto& area = mapSrv.response.area;
  auto overrideOrGlobal = [](auto override, auto global, auto sentinel) {
    return (override != sentinel) ? override : global;
  };

  slic3r_coverage_planner::PlanPath pathSrv;
  pathSrv.request.angle = angle;
  pathSrv.request.outline_count = overrideOrGlobal(area.outline_count, config.outline_count, -1);
  pathSrv.request.outline_overlap_count =
      overrideOrGlobal(area.outline_overlap_count, config.outline_overlap_count, -1);
  pathSrv.request.outline = area.area;
  currentMowingAreaOutline = area.area;
  pathSrv.request.holes = area.obstacles;
  for (const auto& temp_obs : temporary_obstacles) {
    pathSrv.request.holes.push_back(temp_obs);
  }
  pathSrv.request.fill_type = slic3r_coverage_planner::PlanPathRequest::FILL_LINEAR;
  pathSrv.request.outer_offset = std::isnan(area.outline_offset) ? config.outline_offset : area.outline_offset;
  pathSrv.request.distance = config.tool_width;
  if (!pathClient.call(pathSrv)) {
    ROS_ERROR_STREAM("MowingBehavior: Error during coverage planning");
    return false;
  }

  currentMowingPaths = pathSrv.response.paths;

  // Calculate mowing plan digest from the poses
  // TODO: move to slic3r_coverage_planner
  CryptoPP::SHA256 hash;
  byte digest[CryptoPP::SHA256::DIGESTSIZE];
  for (const auto& path : currentMowingPaths) {
    for (const auto& pose_stamped : path.path.poses) {
      hash.Update(reinterpret_cast<const byte*>(&pose_stamped.pose), sizeof(geometry_msgs::Pose));
    }
  }
  hash.Final((byte*)&digest[0]);
  CryptoPP::HexEncoder encoder;
  std::string mowingPlanDigest = "";
  encoder.Attach(new CryptoPP::StringSink(mowingPlanDigest));
  encoder.Put(digest, sizeof(digest));
  encoder.MessageEnd();

  // Proceed to checkpoint?
  if (mowingPlanDigest == currentMowingPlanDigest) {
    ROS_INFO_STREAM("MowingBehavior: Advancing to checkpoint, path: " << currentMowingPath
                                                                      << " index: " << currentMowingPathIndex);
  } else {
    ROS_INFO_STREAM("MowingBehavior: Ignoring checkpoint for plan ("
                    << currentMowingPlanDigest << ") current mowing plan is (" << mowingPlanDigest << ")");
    // Plan has changed so must restart the area
    currentMowingPlanDigest = mowingPlanDigest;
    currentMowingPath = 0;
    currentMowingPathIndex = 0;
  }

  return true;
}

bool MowingBehavior::handle_obstacle_and_replan(double lookahead_dist) {
  auto config = getConfig();
  if (!config.dynamic_obstacle_avoidance) {
    return false;
  }

  if (currentMowingPath >= 0 && currentMowingPath < static_cast<int>(currentMowingPaths.size())) {
    if (currentMowingPaths[currentMowingPath].is_outline && !config.obstacle_detection_on_outlines) {
      ROS_INFO_STREAM("MowingBehavior: Obstacle avoidance disabled on outer lines. Ignoring obstacle on path "
                      << currentMowingPath);
      return false;
    }
  }

  ROS_WARN_STREAM("MowingBehavior: Dynamic Obstacle Avoidance - Obstacle detected on path "
                  << currentMowingPath << " at index " << currentMowingPathIndex << ". Starting grace period of "
                  << config.obstacle_grace_time << "s.");

  // 1. Audio and Event notification
  publishMowerEvent("OBSTACLE_DETECTED", json{{"mowing_path", currentMowingPath},
                                              {"path_index", currentMowingPathIndex},
                                              {"grace_time", config.obstacle_grace_time}});
  broadcastAudioMessage(config.obstacle_audio_message);

  // Get current robot pose
  auto pose_msg = getPose();
  double rx = pose_msg.pose.pose.position.x;
  double ry = pose_msg.pose.pose.position.y;
  tf2::Quaternion q;
  tf2::fromMsg(pose_msg.pose.pose.orientation, q);
  double roll, pitch, initial_yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, initial_yaw);
  double yaw = initial_yaw;

  // Physical mower dimensions dynamically loaded from config / ROS parameters
  double mower_length = config.mower_length > 0.1 ? config.mower_length : 0.46;
  double mower_axle_from_rear = config.mower_axle_from_rear >= 0.0 ? config.mower_axle_from_rear : 0.10;
  double mower_front_x = mower_length - mower_axle_from_rear;  // Front bumper from base_link

  double us_left_x = 0.18, us_left_y = 0.105, us_left_yaw = 0.0;
  double us_right_x = 0.18, us_right_y = -0.105, us_right_yaw = 0.0;
  ros::NodeHandle nh("~");
  nh.getParam("ultrasonic/left/x", us_left_x);
  nh.getParam("ultrasonic/left/y", us_left_y);
  nh.getParam("ultrasonic/left/yaw", us_left_yaw);
  nh.getParam("ultrasonic/right/x", us_right_x);
  nh.getParam("ultrasonic/right/y", us_right_y);
  nh.getParam("ultrasonic/right/yaw", us_right_yaw);

  // 2. Active Sonar Pan Scan during Grace Period
  struct Point2D {
    double x, y;
  };
  std::vector<Point2D> scan_points;

  ros::Time last_left_scan_time(0.0), last_right_scan_time(0.0);
  auto collect_samples = [&](double cur_rx, double cur_ry, double cur_yaw) {
    ros::Time t_sample = ros::Time::now();
    double max_scan_range = std::max(1.2, config.obstacle_detection_distance + 0.2);
    if (us_left_state_subscriber.hasMessage()) {
      ros::Time msg_time = us_left_state_subscriber.getMessageTime();
      if (msg_time != last_left_scan_time) {
        last_left_scan_time = msg_time;
        auto msg = us_left_state_subscriber.getMessage();
        double age = (t_sample - msg_time).toSec();
        if (age < 0.6 && msg.range > 0.02 && msg.range < max_scan_range && !std::isnan(msg.range) &&
            !std::isinf(msg.range)) {
          double lx = us_left_x + msg.range * std::cos(us_left_yaw);
          double ly = us_left_y + msg.range * std::sin(us_left_yaw);
          double mx = cur_rx + lx * std::cos(cur_yaw) - ly * std::sin(cur_yaw);
          double my = cur_ry + lx * std::sin(cur_yaw) + ly * std::cos(cur_yaw);
          scan_points.push_back({mx, my});
        }
      }
    }
    if (us_right_state_subscriber.hasMessage()) {
      ros::Time msg_time = us_right_state_subscriber.getMessageTime();
      if (msg_time != last_right_scan_time) {
        last_right_scan_time = msg_time;
        auto msg = us_right_state_subscriber.getMessage();
        double age = (t_sample - msg_time).toSec();
        if (age < 0.6 && msg.range > 0.02 && msg.range < max_scan_range && !std::isnan(msg.range) &&
            !std::isinf(msg.range)) {
          double lx = us_right_x + msg.range * std::cos(us_right_yaw);
          double ly = us_right_y + msg.range * std::sin(us_right_yaw);
          double mx = cur_rx + lx * std::cos(cur_yaw) - ly * std::sin(cur_yaw);
          double my = cur_ry + lx * std::sin(cur_yaw) + ly * std::cos(cur_yaw);
          scan_points.push_back({mx, my});
        }
      }
    }
  };

  ros::Time grace_start = ros::Time::now();
  ros::Rate sweep_rate(40);  // 40 Hz loop for smooth motion and dense sonar sampling

  if (config.enable_obstacle_pan_scan) {
    ROS_INFO_STREAM("MowingBehavior: Starting active ultrasonic pan scan (+/- " << config.obstacle_pan_angle_deg
                                                                                << " deg)...");
    double pan_rad = config.obstacle_pan_angle_deg * (M_PI / 180.0);
    double pan_speed = std::max(1.0, std::min(2.5, config.obstacle_pan_speed));

    enum PanPhase { PAN_LEFT, PAN_RIGHT, RETURN_CENTER, WAIT_REMAINDER, PAN_DONE };
    PanPhase phase = PAN_LEFT;

    while (ros::ok() && (ros::Time::now() - grace_start).toSec() < config.obstacle_grace_time && phase != PAN_DONE) {
      if (aborted || requested_pause_flag || skip_area || skip_path) {
        setCmdVel(0.0, 0.0);
        return false;
      }

      auto cur_pose_msg = getPose();
      double cur_rx = cur_pose_msg.pose.pose.position.x;
      double cur_ry = cur_pose_msg.pose.pose.position.y;
      tf2::Quaternion cur_q;
      tf2::fromMsg(cur_pose_msg.pose.pose.orientation, cur_q);
      double cur_r, cur_p, cur_yaw;
      tf2::Matrix3x3(cur_q).getRPY(cur_r, cur_p, cur_yaw);

      // Collect ultrasonic sample in map frame
      collect_samples(cur_rx, cur_ry, cur_yaw);

      // Angle error relative to initial yaw
      double angle_diff = cur_yaw - initial_yaw;
      while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
      while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;

      switch (phase) {
        case PAN_LEFT:
          if (angle_diff < pan_rad) {
            setCmdVel(0.0, pan_speed);
          } else {
            setCmdVel(0.0, 0.0);
            phase = PAN_RIGHT;
          }
          break;

        case PAN_RIGHT:
          if (angle_diff > -pan_rad) {
            setCmdVel(0.0, -pan_speed);
          } else {
            setCmdVel(0.0, 0.0);
            phase = RETURN_CENTER;
          }
          break;

        case RETURN_CENTER:
          if (std::abs(angle_diff) > 0.05) {
            setCmdVel(0.0, (angle_diff < 0) ? pan_speed : -pan_speed);
          } else {
            setCmdVel(0.0, 0.0);
            phase = WAIT_REMAINDER;
          }
          break;

        case WAIT_REMAINDER:
          setCmdVel(0.0, 0.0);
          if ((ros::Time::now() - grace_start).toSec() >= config.obstacle_grace_time) {
            phase = PAN_DONE;
          }
          break;

        case PAN_DONE: break;
      }
      sweep_rate.sleep();
    }
    setCmdVel(0.0, 0.0);
  } else {
    // Static wait if pan scan disabled
    while (ros::ok() && (ros::Time::now() - grace_start).toSec() < config.obstacle_grace_time) {
      if (aborted || requested_pause_flag || skip_area || skip_path) {
        return false;
      }
      sweep_rate.sleep();
    }
    collect_samples(rx, ry, initial_yaw);
  }

  // 3. Grace time expired. Recalculate plan with temporary exclusion zone.
  double x_front_edge = mower_front_x + 0.15;
  double y_local = 0.0;
  double obs_heading = initial_yaw;
  bool scan_success = false;

  if (scan_points.size() >= 3) {
    // 1. Transform all points into initial robot local frame (rx, ry, initial_yaw)
    std::vector<double> local_xs, local_ys;
    for (const auto& pt : scan_points) {
      double dx = pt.x - rx;
      double dy = pt.y - ry;
      double lx = dx * cos(initial_yaw) + dy * sin(initial_yaw);
      double ly = -dx * sin(initial_yaw) + dy * cos(initial_yaw);
      // Filter reasonable window in front of mower (from sensor mounting to 1.5m, lateral +/- 1.0m)
      if (lx >= 0.15 && lx <= mower_front_x + 1.5 && std::abs(ly) <= 1.0) {
        local_xs.push_back(lx);
        local_ys.push_back(ly);
      }
    }

    if (local_xs.size() >= 3) {
      // Find closest front edge
      double min_x = *std::min_element(local_xs.begin(), local_xs.end());
      x_front_edge = std::max(mower_front_x + 0.05, min_x);

      // Depth-gate points to only include samples belonging to the obstacle front (within 35cm of min_x).
      // This prevents distant background reflections (e.g. at 1.0m-1.5m on the clear side) from pulling
      // y_local to the center or skewing the PCA line fit!
      std::vector<double> obs_xs, obs_ys;
      for (size_t i = 0; i < local_xs.size(); i++) {
        if (local_xs[i] <= min_x + 0.35) {
          obs_xs.push_back(local_xs[i]);
          obs_ys.push_back(local_ys[i]);
        }
      }
      if (obs_xs.size() < 2) {
        obs_xs = local_xs;
        obs_ys = local_ys;
      }

      // Find lateral center from obstacle cluster
      double min_y = *std::min_element(obs_ys.begin(), obs_ys.end());
      double max_y = *std::max_element(obs_ys.begin(), obs_ys.end());
      double mean_y = std::accumulate(obs_ys.begin(), obs_ys.end(), 0.0) / obs_ys.size();
      y_local = 0.5 * (min_y + max_y) * 0.5 + 0.5 * mean_y;  // Robust blend of midpoint and mean

      // 2. Line Fit (PCA / Total Least Squares) in map frame to get accurate surface tilt
      double sum_mx = 0.0, sum_my = 0.0;
      for (size_t i = 0; i < obs_xs.size(); i++) {
        double mx = rx + obs_xs[i] * cos(initial_yaw) - obs_ys[i] * sin(initial_yaw);
        double my = ry + obs_xs[i] * sin(initial_yaw) + obs_ys[i] * cos(initial_yaw);
        sum_mx += mx;
        sum_my += my;
      }
      double mean_mx = sum_mx / obs_xs.size();
      double mean_my = sum_my / obs_xs.size();

      double Sxx = 0.0, Syy = 0.0, Sxy = 0.0;
      for (size_t i = 0; i < obs_xs.size(); i++) {
        double mx = rx + obs_xs[i] * cos(initial_yaw) - obs_ys[i] * sin(initial_yaw);
        double my = ry + obs_xs[i] * sin(initial_yaw) + obs_ys[i] * cos(initial_yaw);
        double dx = mx - mean_mx;
        double dy = my - mean_my;
        Sxx += dx * dx;
        Syy += dy * dy;
        Sxy += dx * dy;
      }

      // Principal tangent angle of obstacle surface
      double tangent_angle = 0.5 * std::atan2(2.0 * Sxy, Sxx - Syy);
      double norm1 = tangent_angle + M_PI / 2.0;
      double norm2 = tangent_angle - M_PI / 2.0;

      auto angle_diff_fn = [](double a, double b) {
        double d = a - b;
        while (d > M_PI) d -= 2.0 * M_PI;
        while (d < -M_PI) d += 2.0 * M_PI;
        return std::abs(d);
      };

      double fitted_normal = (angle_diff_fn(norm1, initial_yaw) < angle_diff_fn(norm2, initial_yaw)) ? norm1 : norm2;
      double delta_tilt = fitted_normal - initial_yaw;
      while (delta_tilt > M_PI) delta_tilt -= 2.0 * M_PI;
      while (delta_tilt < -M_PI) delta_tilt += 2.0 * M_PI;
      delta_tilt = std::max(-M_PI / 4.0, std::min(M_PI / 4.0, delta_tilt));

      obs_heading = initial_yaw + delta_tilt;
      scan_success = true;

      ROS_INFO_STREAM("MowingBehavior: Pan scan processed "
                      << local_xs.size() << " sonar points -> x_front=" << x_front_edge << "m, y_local=" << y_local
                      << "m (span [" << min_y << ", " << max_y << "]m), tilt=" << (delta_tilt * 180.0 / M_PI)
                      << "deg, obs_heading=" << (obs_heading * 180.0 / M_PI) << "deg");
    }
  }

  // Fallback to static triangulation if pan scan yielded too few points
  if (!scan_success) {
    bool left_detected = false, right_detected = false;
    double left_range = 2.0, right_range = 2.0;
    ros::Time now = ros::Time::now();
    double max_scan_range = std::max(1.2, config.obstacle_detection_distance + 0.2);

    if (us_left_state_subscriber.hasMessage()) {
      auto msg = us_left_state_subscriber.getMessage();
      double age = (now - us_left_state_subscriber.getMessageTime()).toSec();
      if (age < 2.0 && msg.range > 0.02 && msg.range < max_scan_range && !std::isnan(msg.range) &&
          !std::isinf(msg.range)) {
        left_range = msg.range;
        left_detected = true;
      }
    }

    if (us_right_state_subscriber.hasMessage()) {
      auto msg = us_right_state_subscriber.getMessage();
      double age = (now - us_right_state_subscriber.getMessageTime()).toSec();
      if (age < 2.0 && msg.range > 0.02 && msg.range < max_scan_range && !std::isnan(msg.range) &&
          !std::isinf(msg.range)) {
        right_range = msg.range;
        right_detected = true;
      }
    }

    if (!left_detected && !right_detected) {
      ROS_INFO_STREAM(
          "MowingBehavior: No obstacle confirmed in front during stationary check (transient reflection). "
          "Resuming path without adding obstacle.");
      return false;
    }

    if (left_detected && right_detected) {
      double obs_x_l = us_left_x + left_range * std::cos(us_left_yaw);
      double obs_y_l = us_left_y + left_range * std::sin(us_left_yaw);
      double obs_x_r = us_right_x + right_range * std::cos(us_right_yaw);
      double obs_y_r = us_right_y + right_range * std::sin(us_right_yaw);

      double dx_surf = obs_x_l - obs_x_r;
      double dy_surf = obs_y_l - obs_y_r;
      double delta_yaw = std::atan2(dx_surf, dy_surf);
      delta_yaw = std::max(-M_PI / 4.0, std::min(M_PI / 4.0, delta_yaw));
      obs_heading = initial_yaw - delta_yaw;

      double x_min = std::min(obs_x_l, obs_x_r);
      x_front_edge = std::max(mower_front_x + 0.05, x_min);

      double diff = right_range - left_range;
      double t = 0.5 - std::max(-0.5, std::min(0.5, diff * 1.5));
      y_local = obs_y_l * (1.0 - t) + obs_y_r * t;
    } else if (left_detected) {
      double obs_x_l = us_left_x + left_range * std::cos(us_left_yaw);
      double obs_y_l = us_left_y + left_range * std::sin(us_left_yaw);
      x_front_edge = std::max(mower_front_x + 0.05, obs_x_l);
      y_local = obs_y_l;
    } else if (right_detected) {
      double obs_x_r = us_right_x + right_range * std::cos(us_right_yaw);
      double obs_y_r = us_right_y + right_range * std::sin(us_right_yaw);
      x_front_edge = std::max(mower_front_x + 0.05, obs_x_r);
      y_local = obs_y_r;
    }
  }

  double r = config.obstacle_exclusion_radius;

  // The beginning of the obstacle box is ALWAYS fixed at x_front_edge (the detected surface),
  // completely independent of the box size (r)!
  // Front center of obstacle in map coordinates:
  double front_center_x = rx + x_front_edge * cos(yaw) - y_local * sin(yaw);
  double front_center_y = ry + x_front_edge * sin(yaw) + y_local * cos(yaw);

  // Unit vectors oriented along the obstacle's estimated surface normal and tangent:
  double u_obs_fwd_x = cos(obs_heading);
  double u_obs_fwd_y = sin(obs_heading);
  double u_obs_left_x = -sin(obs_heading);
  double u_obs_left_y = cos(obs_heading);

  // Center of the obstacle box in map coordinates:
  double obs_x = front_center_x + r * u_obs_fwd_x;
  double obs_y = front_center_y + r * u_obs_fwd_y;

  // Create oriented rectangle polygon:
  // Front edge is anchored at front_center, rear edge is 2*r along u_obs_fwd, lateral span is +/- r along u_obs_left
  geometry_msgs::Polygon obs_poly;
  geometry_msgs::Point32 p;

  // 1. Front-Right
  p.x = front_center_x - r * u_obs_left_x;
  p.y = front_center_y - r * u_obs_left_y;
  obs_poly.points.push_back(p);

  // 2. Back-Right
  p.x = front_center_x - r * u_obs_left_x + 2.0 * r * u_obs_fwd_x;
  p.y = front_center_y - r * u_obs_left_y + 2.0 * r * u_obs_fwd_y;
  obs_poly.points.push_back(p);

  // 3. Back-Left
  p.x = front_center_x + r * u_obs_left_x + 2.0 * r * u_obs_fwd_x;
  p.y = front_center_y + r * u_obs_left_y + 2.0 * r * u_obs_fwd_y;
  obs_poly.points.push_back(p);

  // 4. Front-Left
  p.x = front_center_x + r * u_obs_left_x;
  p.y = front_center_y + r * u_obs_left_y;
  obs_poly.points.push_back(p);

  // 1. Boundary check: If the obstacle is located outside the active mowing area, ignore it!
  // (e.g. Ultrasonic sensor detected a fence, hedge, or wall outside the lawn perimeter)
  if (!currentMowingAreaOutline.points.empty()) {
    bool in_area = isPointInPolygon(obs_x, obs_y, currentMowingAreaOutline.points);
    if (!in_area) {
      for (const auto& pt : obs_poly.points) {
        if (isPointInPolygon(pt.x, pt.y, currentMowingAreaOutline.points)) {
          in_area = true;
          break;
        }
      }
    }
    if (!in_area) {
      ROS_WARN_STREAM("MowingBehavior: Detected obstacle at ("
                      << obs_x << ", " << obs_y
                      << ") is outside the mowing area boundary. Ignoring to prevent unnecessary detours.");
      return false;
    }
  }

  // 2. Path Slicing & Obstacle Avoidance:
  // Check if the current path is actually blocked by this obstacle
  if (currentMowingPath >= 0 && currentMowingPath < static_cast<int>(currentMowingPaths.size())) {
    auto& cur_path = currentMowingPaths[currentMowingPath];

    // Scan current path starting from currentMowingPathIndex to find the immediate blocked segment.
    // Limit search window to the immediate vicinity (at most 40 poses / ~3-4 meters ahead)
    // so we NEVER falsely match a future outline loop or subsequent parallel sweep hundreds of poses later!
    int block_start = -1;
    int block_end = -1;
    size_t search_limit = std::min(cur_path.path.poses.size(), static_cast<size_t>(currentMowingPathIndex + 40));

    for (size_t i = currentMowingPathIndex; i < search_limit; i++) {
      const auto& pt = cur_path.path.poses[i].pose.position;
      // Project pose into robot local coordinate frame
      double dx = pt.x - rx;
      double dy = pt.y - ry;
      double lx = dx * cos(yaw) + dy * sin(yaw);
      double ly = -dx * sin(yaw) + dy * cos(yaw);

      // Distance to obstacle center
      double d_to_obs = std::hypot(pt.x - obs_x, pt.y - obs_y);

      // A pose is blocked if it is inside the oriented obstacle box OR within (r + 0.25m) of the obstacle
      bool inside = ((lx >= x_front_edge - 0.10) && (lx <= x_front_edge + 2.0 * r + 0.30) &&
                     (std::abs(ly - y_local) <= r + 0.25)) ||
                    (d_to_obs <= r + 0.25);

      if (inside) {
        if (block_start == -1) {
          block_start = static_cast<int>(i);
        }
        block_end = static_cast<int>(i);
      } else if (block_start != -1) {
        // Exited the immediate obstacle zone along the path!
        break;
      }
    }

    // If the obstacle does not block the current path (e.g. obstacle is off to the side), do not slice or detour!
    if (block_start == -1) {
      ROS_INFO_STREAM("MowingBehavior: Detected obstacle does not intersect current path. Continuing path execution.");
      return false;
    }

    // Obstacle is inside the mowing area AND blocks the current path -> Register obstacle and slice path!
    broadcastAudioMessage("Rerouting around obstacle.");
    publishMowerEvent("OBSTACLE_REROUTING");
    temporary_obstacles.push_back(obs_poly);

    json obs_poly_json = json::array();
    for (const auto& pt : obs_poly.points) {
      obs_poly_json.push_back(json{{"x", pt.x}, {"y", pt.y}});
    }

    json all_obstacles_json = json::array();
    for (const auto& poly : temporary_obstacles) {
      json poly_pts = json::array();
      for (const auto& pt : poly.points) {
        poly_pts.push_back(json{{"x", pt.x}, {"y", pt.y}});
      }
      all_obstacles_json.push_back(
          json{{"x", obs_x}, {"y", obs_y}, {"radius", r}, {"heading", obs_heading}, {"polygon", poly_pts}});
    }

    publishMowerEvent("OBSTACLE_ADDED", json{{"obstacle_x", obs_x},
                                             {"obstacle_y", obs_y},
                                             {"radius", r},
                                             {"heading", obs_heading},
                                             {"polygon", obs_poly_json},
                                             {"obstacles", all_obstacles_json}});

    // Publish retained temporary obstacles topic to MQTT so late-joining Web GUIs display them immediately
    publishMqtt("temporary_obstacles/json", all_obstacles_json, true);

    // Publish temporary obstacle to mower_map_service so global_costmap marks it as occupied
    add_dynamic_obstacle_pub.publish(obs_poly);
    just_avoided_obstacle = true;
    ros::Duration(1.0).sleep();

    ROS_WARN_STREAM("MowingBehavior: Added temporary no-mow zone at (" << obs_x << ", " << obs_y << ") with radius "
                                                                       << r << "m to costmap. Slicing current path.");

    ROS_INFO_STREAM("MowingBehavior: Slicing path " << currentMowingPath << " around obstacle: blocked poses "
                                                    << block_start << " to " << block_end << " of "
                                                    << cur_path.path.poses.size());

    // Check if there are poses behind the obstacle to mow (require at least 3 poses / 30cm)
    slic3r_coverage_planner::Path remainder_path;
    bool has_remainder = (block_end + 3 < static_cast<int>(cur_path.path.poses.size()));

    if (has_remainder) {
      remainder_path.is_outline = cur_path.is_outline;
      remainder_path.path.header = cur_path.path.header;
      remainder_path.path.poses.assign(cur_path.path.poses.begin() + block_end + 1, cur_path.path.poses.end());
      ROS_INFO_STREAM("MowingBehavior: Created continuation path with " << remainder_path.path.poses.size()
                                                                        << " poses behind obstacle.");
    }

    // Truncate current path before the obstacle
    if (block_start > 0) {
      cur_path.path.poses.resize(block_start);
    }

    // Insert remainder path right after current path so the robot navigates around the obstacle to finish this line!
    if (has_remainder && !remainder_path.path.poses.empty()) {
      currentMowingPaths.insert(currentMowingPaths.begin() + currentMowingPath + 1, remainder_path);
    }

    // Advance to remainder path (or next path if no remainder)
    currentMowingPath++;
    currentMowingPathIndex = 0;
    ROS_INFO_STREAM("MowingBehavior: Bypassing obstacle to path " << currentMowingPath << " of "
                                                                  << currentMowingPaths.size());
    return true;
  }
  return false;
}

int getCurrentMowPathIndex() {
  ftc_local_planner::PlannerGetProgress progressSrv;
  int currentIndex = -1;
  if (pathProgressClient.call(progressSrv)) {
    currentIndex = progressSrv.response.index;
  } else {
    ROS_ERROR("MowingBehavior: getMowIndex() - Error getting progress from FTC planner");
  }
  return (currentIndex);
}

void printNavState(int state) {
  switch (state) {
    case actionlib::SimpleClientGoalState::PENDING: ROS_INFO(">>> State: Pending <<<"); break;
    case actionlib::SimpleClientGoalState::ACTIVE: ROS_INFO(">>> State: Active <<<"); break;
    case actionlib::SimpleClientGoalState::RECALLED: ROS_INFO(">>> State: Recalled <<<"); break;
    case actionlib::SimpleClientGoalState::REJECTED: ROS_INFO(">>> State: Rejected <<<"); break;
    case actionlib::SimpleClientGoalState::PREEMPTED: ROS_INFO(">>> State: Preempted <<<"); break;
    case actionlib::SimpleClientGoalState::ABORTED: ROS_INFO(">>> State: Aborted <<<"); break;
    case actionlib::SimpleClientGoalState::SUCCEEDED: ROS_INFO(">>> State: Succeeded <<<"); break;
    case actionlib::SimpleClientGoalState::LOST: ROS_INFO(">>> State: Lost <<<"); break;
    default: ROS_INFO(">>> State: Unknown Hu ? <<<"); break;
  }
}

namespace {
// Returns the names of the recovery behaviors configured on move_base_flex
// (its "recovery_behaviors" param), in order, or an empty list if none are
// configured. This avoids hardcoding behavior names and makes recovery a no-op
// when MBF has no recovery configured.
std::vector<std::string> getConfiguredRecoveryBehaviors() {
  std::vector<std::string> names;
  XmlRpc::XmlRpcValue behaviors;
  if (!ros::param::get("/move_base_flex/recovery_behaviors", behaviors)) {
    return names;
  }
  if (behaviors.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    return names;
  }
  for (int i = 0; i < behaviors.size(); i++) {
    XmlRpc::XmlRpcValue& b = behaviors[i];
    if (b.getType() == XmlRpc::XmlRpcValue::TypeStruct && b.hasMember("name")) {
      names.push_back(static_cast<std::string>(b["name"]));
    }
  }
  return names;
}
}  // namespace

/// @return true if spinup succeeded or spinup is disabled, false if we should abort/exit
bool MowingBehavior::wait_for_mower_spinup() {
  if (config.mower_spinup_rpm <= 0) {
    return true;  // spinup check disabled
  }

  ROS_INFO_STREAM("MowingBehavior: (MOW) Waiting for mower motor to reach " << config.mower_spinup_rpm
                                                                            << " RPM before driving");
  ros::Time spinup_start = ros::Time::now();
  ros::Rate check_rate(10);
  while (ros::ok()) {
    if (aborted || requested_pause_flag || skip_area || skip_path) {
      return false;
    }
    auto last_status = status_state_subscriber.getMessage();
    if (std::abs(last_status.mower_motor_rpm) >= config.mower_spinup_rpm) {
      ROS_INFO_STREAM("MowingBehavior: (MOW) Mower motor reached " << last_status.mower_motor_rpm << " RPM after "
                                                                   << (ros::Time::now() - spinup_start).toSec() << "s");
      return true;
    }
    if (ros::Time::now() - spinup_start > ros::Duration(config.mower_spinup_timeout)) {
      ROS_ERROR_STREAM("MowingBehavior: (MOW) Mower motor failed to reach " << config.mower_spinup_rpm << " RPM within "
                                                                            << config.mower_spinup_timeout
                                                                            << "s. Entering emergency.");
      publishMowerEvent("MOW_MOTOR_SPINUP_FAILED");
      mowerEnabled = false;
      setEmergencyMode(EmergencyReason::MOWER_RPM_TIMEOUT);
      return false;
    }
    check_rate.sleep();
  }
  return false;
}

bool MowingBehavior::execute_mowing_plan() {
  int first_point_attempt_counter = 0;
  int first_point_trim_counter = 0;
  ros::Time paused_time(0.0);

  // loop through all mowingPaths to execute the plan fully.
  while (currentMowingPath < currentMowingPaths.size() && ros::ok() && !aborted) {
    ////////////////////////////////////////////////
    // PAUSE HANDLING
    ////////////////////////////////////////////////
    if (requested_pause_flag) {  // pause was requested
      paused = true;
      mowerEnabled = false;
      u_int8_t last_requested_pause_flags = 0;
      while (requested_pause_flag && !aborted)  // while emergency and/or manual pause not asked to continue, we wait
      {
        if (last_requested_pause_flags != requested_pause_flag) {
          update_actions();
        }
        last_requested_pause_flags = requested_pause_flag;

        std::string pause_reason = "";
        if (requested_pause_flag & pauseType::PAUSE_EMERGENCY) {
          pause_reason += "on EMERGENCY";
          if (requested_pause_flag & pauseType::PAUSE_MANUAL) {
            pause_reason += " and ";
          }
        }
        if (requested_pause_flag & pauseType::PAUSE_MANUAL) {
          pause_reason += "waiting for CONTINUE";
        }
        ROS_INFO_STREAM_THROTTLE(30, "MowingBehavior: PAUSED (" << pause_reason << ")");
        ros::Rate r(1.0);
        r.sleep();
      }
      // we will drop into paused, thus will also wait for GPS to be valid again
    }
    if (paused) {
      paused_time = ros::Time::now();
      while (!this->hasGoodGPS() && !aborted)  // while no good GPS we wait
      {
        ROS_INFO_STREAM("MowingBehavior: PAUSED (" << (ros::Time::now() - paused_time).toSec()
                                                   << "s) (waiting for GPS)");
        ros::Rate r(1.0);
        r.sleep();
      }
      ROS_INFO_STREAM("MowingBehavior: CONTINUING");
      paused = false;
      update_actions();
    }

    auto& path = currentMowingPaths[currentMowingPath];
    ROS_INFO_STREAM("MowingBehavior: Path segment length: " << path.path.poses.size() << " poses.");

    // Check if path is empty. If so, directly skip it
    if (currentMowingPathIndex >= path.path.poses.size()) {
      ROS_INFO_STREAM("MowingBehavior: Skipping empty path.");
      currentMowingPath++;
      currentMowingPathIndex = 0;
      continue;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    // DRIVE TO THE FIRST POINT OF THE MOW PATH
    //
    // * we have n attempts, if we fail we go to pause() mode because most likely it was GPS problems that
    //   prevented us from reaching the inital pose
    // * after n attempts, we fail the mow area and skip to the next one
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    auto current_pose_msg = getPose();
    double current_rx = current_pose_msg.pose.pose.position.x;
    double current_ry = current_pose_msg.pose.pose.position.y;
    const auto& target_start_pt = path.path.poses[currentMowingPathIndex].pose.position;
    double dist_to_start = std::hypot(current_rx - target_start_pt.x, current_ry - target_start_pt.y);

    if (dist_to_start > 0.35 || just_avoided_obstacle) {
      bool is_obstacle_detour = just_avoided_obstacle;
      if (just_avoided_obstacle) {
        ROS_INFO_STREAM("MowingBehavior: (FIRST POINT) Detouring around newly placed dynamic obstacle (dist="
                        << dist_to_start << "m)");
        just_avoided_obstacle = false;
      } else {
        ROS_INFO_STREAM("MowingBehavior: (FIRST POINT)  Moving to path segment starting point (dist=" << dist_to_start
                                                                                                      << "m)");
      }
      if (path.is_outline && getConfig().add_fake_obstacle) {
        mower_map::SetNavPointSrv set_nav_point_srv;
        set_nav_point_srv.request.nav_pose = path.path.poses[currentMowingPathIndex].pose;
        setNavPointClient.call(set_nav_point_srv);
        sleep(1);
      }

      if (is_obstacle_detour && getConfig().mow_during_obstacle_detour) {
        ROS_INFO_STREAM("MowingBehavior: (FIRST POINT) Keeping mower motor enabled to mow detour around obstacle.");
        mowerEnabled = true;
        wait_for_mower_spinup();
      }

      mbf_msgs::MoveBaseGoal moveBaseGoal;
      moveBaseGoal.target_pose = path.path.poses[currentMowingPathIndex];
      moveBaseGoal.controller = "FTCPlanner";
      mbfClient->sendGoal(moveBaseGoal);
      sleep(1);
      actionlib::SimpleClientGoalState current_status(actionlib::SimpleClientGoalState::PENDING);
      ros::Rate r(10);

      // wait for path execution to finish
      while (ros::ok()) {
        current_status = mbfClient->getState();
        if (current_status.state_ == actionlib::SimpleClientGoalState::ACTIVE ||
            current_status.state_ == actionlib::SimpleClientGoalState::PENDING) {
          // path is being executed, everything seems fine.
          // check if we should pause or abort mowing
          if (skip_area) {
            ROS_INFO_STREAM("MowingBehavior: (FIRST POINT) SKIP AREA was requested.");
            publishMowerEvent("AREA_SKIPPED");
            // remove all paths in current area and return true
            mowerEnabled = false;
            mbfClientExePath->cancelAllGoals();
            currentMowingPaths.clear();
            skip_area = false;
            return true;
          }
          if (skip_path) {
            skip_path = false;
            currentMowingPath++;
            currentMowingPathIndex = 0;
            return false;
          }
          if (aborted) {
            ROS_INFO_STREAM("MowingBehavior: (FIRST POINT) ABORT was requested - stopping path execution.");
            mbfClientExePath->cancelAllGoals();
            mowerEnabled = false;
            return false;
          }
          if (requested_pause_flag) {
            ROS_INFO_STREAM("MowingBehavior: (FIRST POINT) PAUSE was requested - stopping path execution.");
            mbfClientExePath->cancelAllGoals();
            mowerEnabled = false;
            return false;
          }
        } else {
          ROS_INFO_STREAM("MowingBehavior: (FIRST POINT)  Got status "
                          << current_status.state_ << " from MBF/FTCPlanner -> Stopping path execution.");
          // we're done, break out of the loop
          break;
        }
        r.sleep();
      }

      first_point_attempt_counter++;
      if (current_status.state_ != actionlib::SimpleClientGoalState::SUCCEEDED) {
        mowerEnabled = false;
        // we cannot reach the start point
        ROS_ERROR_STREAM("MowingBehavior: (FIRST POINT) - Could not reach goal (first point). Planner Status was: "
                         << current_status.state_);
        // we have 3 attempts to get to the start pose of the mowing area
        if (first_point_attempt_counter < config.max_first_point_attempts) {
          ROS_WARN_STREAM("MowingBehavior: (FIRST POINT) - Attempt " << first_point_attempt_counter << " / "
                                                                     << config.max_first_point_attempts
                                                                     << " Making a little pause ...");
          paused = true;
          update_actions();
        } else {
          // We failed to reach the first point in the mow path by simply repeating the drive to process
          // So now we will trim the path by removing the first pose
          if (first_point_trim_counter < config.max_first_point_trim_attempts) {
            // We try now to remove the first point so the 2nd, 3rd etc point becomes our target
            // mow path points are offset by 10cm
            ROS_WARN_STREAM("MowingBehavior: (FIRST POINT) - Attempt "
                            << first_point_trim_counter << " / " << config.max_first_point_trim_attempts
                            << " Trimming first point off the beginning of the mow path.");
            currentMowingPathIndex++;
            first_point_trim_counter++;
            first_point_attempt_counter = 0;  // give it another <config.max_first_point_attempts> attempts
          } else {
            // Unable to reach the start of this mow path after multiple trim attempts (obstacle blocking the start
            // point). Skip this entire mow line and proceed to the next one!
            ROS_WARN_STREAM("MowingBehavior: (FIRST POINT) Max retries reached for path "
                            << currentMowingPath << " in area " << currentMowingArea
                            << " - skipping to next mow path.");
            currentMowingPath++;
            currentMowingPathIndex = 0;
            first_point_attempt_counter = 0;
            first_point_trim_counter = 0;
          }
        }
        continue;
      }

      mower_map::ClearNavPointSrv clear_nav_point_srv;
      clearNavPointClient.call(clear_nav_point_srv);

      // we have reached the start pose of the mow area, reset error handling values
      first_point_attempt_counter = 0;
      first_point_trim_counter = 0;
    } else {
      ROS_INFO_STREAM("MowingBehavior: (FIRST POINT) Already at path starting point (dist="
                      << dist_to_start << "m) - skipping drive to start.");
      first_point_attempt_counter = 0;
      first_point_trim_counter = 0;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Execute the path segment and either drop it if we finished it successfully or trim it if we were aborted
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    {
      // enable mower (only when we reach the start not on the way to mowing already)
      bool mower_enabled_last = mowerEnabled;
      mowerEnabled = true;

      // Wait for mower motor to spin up to target RPM (if configured and previously disabled)
      if (!mower_enabled_last && !wait_for_mower_spinup()) {
        return false;
      }

      mbf_msgs::ExePathGoal exePathGoal;
      nav_msgs::Path exePath;
      exePath.header = path.path.header;
      exePath.poses = std::vector<geometry_msgs::PoseStamped>(path.path.poses.begin() + currentMowingPathIndex,
                                                              path.path.poses.end());
      int exePathStartIndex = currentMowingPathIndex;
      exePathGoal.path = exePath;
      exePathGoal.angle_tolerance = 5.0 * (M_PI / 180.0);
      exePathGoal.dist_tolerance = 0.2;
      exePathGoal.tolerance_from_action = true;
      exePathGoal.controller = "FTCPlanner";

      ROS_INFO_STREAM("MowingBehavior: (MOW) First point reached - Executing mow path with "
                      << path.path.poses.size() << " poses, from index " << exePathStartIndex);
      mbfClientExePath->sendGoal(exePathGoal);
      sleep(1);
      actionlib::SimpleClientGoalState current_status(actionlib::SimpleClientGoalState::PENDING);
      ros::Rate r(10);
      int consecutive_obstacle_detections = 0;
      double us_left_x = 0.18, us_left_y = 0.105, us_left_yaw = 0.0;
      double us_right_x = 0.18, us_right_y = -0.105, us_right_yaw = 0.0;
      ros::NodeHandle nh_sonar("~");
      nh_sonar.getParam("ultrasonic/left/x", us_left_x);
      nh_sonar.getParam("ultrasonic/left/y", us_left_y);
      nh_sonar.getParam("ultrasonic/left/yaw", us_left_yaw);
      nh_sonar.getParam("ultrasonic/right/x", us_right_x);
      nh_sonar.getParam("ultrasonic/right/y", us_right_y);
      nh_sonar.getParam("ultrasonic/right/yaw", us_right_yaw);

      // wait for path execution to finish
      while (ros::ok()) {
        current_status = mbfClientExePath->getState();
        if (current_status.state_ == actionlib::SimpleClientGoalState::ACTIVE ||
            current_status.state_ == actionlib::SimpleClientGoalState::PENDING) {
          // path is being executed, everything seems fine.
          // check if we should pause or abort mowing
          if (skip_area) {
            ROS_INFO_STREAM("MowingBehavior: (MOW) SKIP AREA was requested.");
            publishMowerEvent("AREA_SKIPPED");
            // remove all paths in current area and return true
            mowerEnabled = false;
            currentMowingPaths.clear();
            skip_area = false;
            return true;
          }
          if (skip_path) {
            skip_path = false;
            currentMowingPath++;
            currentMowingPathIndex = 0;
            return false;
          }
          if (aborted) {
            ROS_INFO_STREAM("MowingBehavior: (MOW) ABORT was requested - stopping path execution.");
            mbfClientExePath->cancelAllGoals();
            mowerEnabled = false;
            break;  // Trim path
          }
          if (requested_pause_flag) {
            ROS_INFO_STREAM("MowingBehavior: (MOW) PAUSE was requested - stopping path execution.");
            mbfClientExePath->cancelAllGoals();
            mowerEnabled = false;
            break;  // Trim path
          }
          if (current_status.state_ == actionlib::SimpleClientGoalState::ACTIVE) {
            // show progress
            int currentIndex = getCurrentMowPathIndex();
            if (currentIndex != -1) {
              currentMowingPathIndex = exePathStartIndex + currentIndex;
            }
            ROS_INFO_STREAM_THROTTLE(
                5, "MowingBehavior: (MOW) Progress: " << currentMowingPathIndex << "/" << path.path.poses.size());
            if (ros::Time::now() - last_checkpoint > ros::Duration(30.0)) checkpoint();

            // Active ultrasonic obstacle detection while driving
            auto current_cfg = getConfig();
            if (current_cfg.dynamic_obstacle_avoidance &&
                (!path.is_outline || current_cfg.obstacle_detection_on_outlines)) {
              double detect_dist = current_cfg.obstacle_detection_distance;
              double mower_length = current_cfg.mower_length > 0.1 ? current_cfg.mower_length : 0.46;
              double mower_axle_from_rear =
                  current_cfg.mower_axle_from_rear >= 0.0 ? current_cfg.mower_axle_from_rear : 0.10;
              double mower_front_x = mower_length - mower_axle_from_rear;

              ros::Time now = ros::Time::now();
              bool left_detected = false;
              bool right_detected = false;
              double min_obstacle_dist = 999.0;

              // Current robot pose to project sensor echoes into map coordinates
              auto cur_pose_msg = getPose();
              double rx = cur_pose_msg.pose.pose.position.x;
              double ry = cur_pose_msg.pose.pose.position.y;
              tf2::Quaternion q;
              tf2::fromMsg(cur_pose_msg.pose.pose.orientation, q);
              double r_roll, r_pitch, cur_yaw;
              tf2::Matrix3x3(q).getRPY(r_roll, r_pitch, cur_yaw);

              if (us_left_state_subscriber.hasMessage()) {
                auto msg = us_left_state_subscriber.getMessage();
                double age = (now - us_left_state_subscriber.getMessageTime()).toSec();
                if (age < 0.6 && msg.range > 0.02 && msg.range < detect_dist && !std::isnan(msg.range) &&
                    !std::isinf(msg.range)) {
                  double lx = us_left_x + msg.range * std::cos(us_left_yaw);
                  double ly = us_left_y + msg.range * std::sin(us_left_yaw);
                  double mx = rx + lx * std::cos(cur_yaw) - ly * std::sin(cur_yaw);
                  double my = ry + lx * std::sin(cur_yaw) + ly * std::cos(cur_yaw);
                  if (currentMowingAreaOutline.points.empty() ||
                      isPointInPolygon(mx, my, currentMowingAreaOutline.points)) {
                    left_detected = true;
                    min_obstacle_dist = std::min(min_obstacle_dist, static_cast<double>(msg.range));
                  }
                }
              }

              if (us_right_state_subscriber.hasMessage()) {
                auto msg = us_right_state_subscriber.getMessage();
                double age = (now - us_right_state_subscriber.getMessageTime()).toSec();
                if (age < 0.6 && msg.range > 0.02 && msg.range < detect_dist && !std::isnan(msg.range) &&
                    !std::isinf(msg.range)) {
                  double lx = us_right_x + msg.range * std::cos(us_right_yaw);
                  double ly = us_right_y + msg.range * std::sin(us_right_yaw);
                  double mx = rx + lx * std::cos(cur_yaw) - ly * std::sin(cur_yaw);
                  double my = ry + lx * std::sin(cur_yaw) + ly * std::cos(cur_yaw);
                  if (currentMowingAreaOutline.points.empty() ||
                      isPointInPolygon(mx, my, currentMowingAreaOutline.points)) {
                    right_detected = true;
                    min_obstacle_dist = std::min(min_obstacle_dist, static_cast<double>(msg.range));
                  }
                }
              }

              bool any_detected = left_detected || right_detected;
              if (any_detected) {
                consecutive_obstacle_detections++;
                bool trigger_stop = false;
                if ((left_detected && right_detected) && consecutive_obstacle_detections >= 2) {
                  trigger_stop = true;
                } else if (min_obstacle_dist <= mower_front_x + 0.12 && consecutive_obstacle_detections >= 2) {
                  trigger_stop = true;
                } else if (consecutive_obstacle_detections >= 6) {
                  trigger_stop = true;
                }

                if (trigger_stop) {
                  ROS_WARN_STREAM("MowingBehavior: (MOW) Obstacle detected ahead by ultrasonic sensors within "
                                  << min_obstacle_dist << "m (threshold " << detect_dist
                                  << "m)! Stopping path execution to initiate obstacle avoidance.");
                  mbfClientExePath->cancelAllGoals();
                  mowerEnabled = false;
                  break;  // Exit path execution loop to trigger handle_obstacle_and_replan
                }
              } else {
                consecutive_obstacle_detections = 0;
              }
            }
          }
        } else {
          ROS_INFO_STREAM("MowingBehavior: (MOW)  Got status " << current_status.state_
                                                               << " from MBF/FTCPlanner -> Stopping path execution.");
          // we're done, break out of the loop
          break;
        }
        r.sleep();
      }

      // Only skip/trim if goal execution began
      if (current_status.state_ != actionlib::SimpleClientGoalState::PENDING &&
          current_status.state_ != actionlib::SimpleClientGoalState::RECALLED) {
        ROS_INFO_STREAM(">> MowingBehavior: (MOW) PlannerGetProgress currentMowingPathIndex = "
                        << currentMowingPathIndex << " of " << path.path.poses.size());
        printNavState(current_status.state_);
        // if we have fully processed the segment or we have encountered an error, drop the path segment
        /* TODO: we can not trust the SUCCEEDED state because the planner sometimes says suceeded with
            the currentIndex far from the size of the poses ! (BUG in planner ?)
            instead we trust only the currentIndex vs. poses.size() */
        if (currentMowingPathIndex >= path.path.poses.size() ||
            (path.path.poses.size() - currentMowingPathIndex) < 5)  // fully mowed the path ?
        {
          ROS_INFO_STREAM("MowingBehavior: (MOW) Mow path finished, skipping to next mow path.");
          currentMowingPath++;
          currentMowingPathIndex = 0;
          // continue with next segment
        } else {
          // we didnt drive all points in the mow path, so we wait for GPS or execute the recovery behaviors.

          // currentMowingPathIndex might be 0 if we never consumed one of the points, we advance at least 1 point
          if (currentMowingPathIndex == 0) currentMowingPathIndex++;
          if (!requested_pause_flag) {
            auto config = getConfig();
            if (config.dynamic_obstacle_avoidance && (!path.is_outline || config.obstacle_detection_on_outlines)) {
              if (handle_obstacle_and_replan(0.5)) {
                continue;
              }
              // Obstacle check complete (obstacle was outside area or did not block path).
              // Resume path execution directly without running recovery maneuvers or pausing!
              ROS_INFO_STREAM("MowingBehavior: (MOW) Obstacle check complete - resuming path execution.");
              continue;
            }

            // Path following failed and dynamic obstacle avoidance is not handling it.
            // Only execute physical recovery behaviors (like BackwardForwardRecovery)
            // if a physical bumper collision occurred or mower is stuck in emergency,
            // NEVER for ultrasonic non-contact sensor events!
            bool is_physical_emergency =
                emergency_state_subscriber.hasMessage() && (emergency_state_subscriber.getMessage().active_emergency ||
                                                            emergency_state_subscriber.getMessage().latched_emergency);

            if (is_physical_emergency && hasGoodGPS() && !aborted) {
              mowerEnabled = false;
              std::vector<std::string> recoveryBehaviors = getConfiguredRecoveryBehaviors();
              if (recoveryBehaviors.empty()) {
                ROS_INFO_STREAM(
                    "MowingBehavior: (MOW) No recovery behavior configured - "
                    "skipping recovery.");
              } else if (!mbfClientRecovery->waitForServer(ros::Duration(1.0))) {
                ROS_WARN_STREAM(
                    "MowingBehavior: (MOW) Recovery action server unavailable - "
                    "skipping recovery.");
              } else {
                for (const auto& behavior : recoveryBehaviors) {
                  if (aborted) break;
                  mbf_msgs::RecoveryGoal recoveryGoal;
                  recoveryGoal.behavior = behavior;
                  ROS_INFO_STREAM(
                      "MowingBehavior: (MOW) Physical bumper collision/stall detected - running recovery "
                      "behavior '"
                      << behavior << "'.");
                  auto recoveryState = sendGoalAndWaitUnlessAborted(mbfClientRecovery, recoveryGoal);
                  ROS_INFO_STREAM("MowingBehavior: (MOW) Recovery behavior '" << behavior << "' finished with state "
                                                                              << recoveryState.toString());
                  if (recoveryState == actionlib::SimpleClientGoalState::SUCCEEDED) {
                    break;
                  }
                }
              }
            }
            ROS_INFO_STREAM("MowingBehavior: (MOW) PAUSED due to MBF Error at " << currentMowingPathIndex);
            publishMowerEvent("NAVIGATION_ERROR");
            paused = true;
            update_actions();
          }
        }
      }
    }
  }

  mowerEnabled = false;

  // true, if we have executed all paths
  return currentMowingPath >= currentMowingPaths.size();
}

void MowingBehavior::command_home() {
  if (shared_state->active_semiautomatic_task) {
    // We are in semiautomatic task, mark it as manually paused.
    ROS_INFO_STREAM("Manually pausing semiautomatic task");
    auto config = getConfig();
    config.manual_pause_mowing = true;
    setConfig(config);
  }
  if (paused) {
    // Request continue to wait for odom
    this->requestContinue();
    // Then instantly abort i.e. go to dock.
  }
  this->abort();
}

void MowingBehavior::command_start() {
  ROS_INFO_STREAM("MowingBehavior: MANUAL CONTINUE");
  auto config = getConfig();
  if (shared_state->active_semiautomatic_task && config.manual_pause_mowing) {
    // We are in semiautomatic task and paused, user wants to resume, so store that immediately.
    // This way, once we are docked the mower will continue as soon as all other conditions are g2g
    ROS_INFO_STREAM("Resuming semiautomatic task");
    config.manual_pause_mowing = false;
    setConfig(config);
  }
  this->requestContinue();
}

void MowingBehavior::command_s1() {
  ROS_INFO_STREAM("MowingBehavior: MANUAL PAUSED");
  this->requestPause();
}

void MowingBehavior::command_s2() {
  skip_area = true;
}

bool MowingBehavior::redirect_joystick() {
  return false;
}

uint8_t MowingBehavior::get_sub_state() {
  return 0;
}

uint8_t MowingBehavior::get_state() {
  return mower_msgs::HighLevelStatus::HIGH_LEVEL_STATE_AUTONOMOUS;
}

int16_t MowingBehavior::get_current_area() const {
  return currentMowingArea;
}

std::string MowingBehavior::get_current_area_id() const {
  return currentMowingAreaId;
}

std::string MowingBehavior::get_current_area_name() const {
  return currentMowingAreaName;
}

int16_t MowingBehavior::get_current_path() {
  return currentMowingPath;
}

int16_t MowingBehavior::get_current_path_index() {
  return currentMowingPathIndex;
}

MowingBehavior::MowingBehavior() {
  last_checkpoint = ros::Time(0.0);
  xbot_msgs::ActionInfo pause_action;
  pause_action.action_id = "pause";
  pause_action.enabled = false;
  pause_action.action_name = "Pause Mowing";

  xbot_msgs::ActionInfo continue_action;
  continue_action.action_id = "continue";
  continue_action.enabled = false;
  continue_action.action_name = "Continue Mowing";

  xbot_msgs::ActionInfo abort_mowing_action;
  abort_mowing_action.action_id = "abort_mowing";
  abort_mowing_action.enabled = false;
  abort_mowing_action.action_name = "Stop Mowing";

  xbot_msgs::ActionInfo skip_area_action;
  skip_area_action.action_id = "skip_area";
  skip_area_action.enabled = false;
  skip_area_action.action_name = "Skip Area";

  xbot_msgs::ActionInfo skip_path_action;
  skip_path_action.action_id = "skip_path";
  skip_path_action.enabled = false;
  skip_path_action.action_name = "Skip Path";

  actions.clear();
  actions.push_back(pause_action);
  actions.push_back(continue_action);
  actions.push_back(abort_mowing_action);
  actions.push_back(skip_area_action);
  actions.push_back(skip_path_action);
  restore_checkpoint();
}

void MowingBehavior::handle_action(std::string action) {
  if (action == "mower_logic:mowing/pause") {
    ROS_INFO_STREAM("got pause command");
    this->requestPause();
  } else if (action == "mower_logic:mowing/continue") {
    ROS_INFO_STREAM("got continue command");
    this->requestContinue();
  } else if (action == "mower_logic:mowing/abort_mowing") {
    ROS_INFO_STREAM("got abort mowing command");
    command_home();
  } else if (action == "mower_logic:mowing/skip_area") {
    ROS_INFO_STREAM("got skip_area command");
    skip_area = true;
  } else if (action == "mower_logic:mowing/skip_path") {
    ROS_INFO_STREAM("got skip_path command");
    skip_path = true;
  }
  update_actions();
}

void MowingBehavior::checkpoint() {
  rosbag::Bag bag;
  mower_logic::CheckPoint cp;
  cp.job_id = current_job_id;
  cp.currentMowingPath = currentMowingPath;
  cp.currentMowingArea = currentMowingArea;
  cp.currentMowingPathIndex = currentMowingPathIndex;
  cp.currentMowingPlanDigest = currentMowingPlanDigest;
  cp.currentMowingAngleIncrementSum = currentMowingAngleIncrementSum;
  bag.open("checkpoint.bag", rosbag::bagmode::Write);
  bag.write("checkpoint", ros::Time::now(), cp);
  bag.close();
  last_checkpoint = ros::Time::now();
}

bool MowingBehavior::restore_checkpoint() {
  rosbag::Bag bag;
  bool found = false;
  try {
    bag.open("checkpoint.bag");
  } catch (rosbag::BagIOException& e) {
    // Checkpoint does not exist or is corrupt, start at the very beginning
    currentMowingArea = 0;
    currentMowingPath = 0;
    currentMowingPathIndex = 0;
    currentMowingAngleIncrementSum = 0;
    return false;
  }
  {
    rosbag::View view(bag, rosbag::TopicQuery("checkpoint"));
    for (rosbag::MessageInstance const m : view) {
      auto cp = m.instantiate<mower_logic::CheckPoint>();
      if (cp) {
        ROS_INFO_STREAM("Restoring checkpoint for plan ("
                        << cp->currentMowingPlanDigest << ")"
                        << " job: " << cp->job_id << " area: " << cp->currentMowingArea
                        << " path: " << cp->currentMowingPath << " index: " << cp->currentMowingPathIndex
                        << " angle increment sum: " << cp->currentMowingAngleIncrementSum);
        current_job_id = cp->job_id;
        currentMowingPath = cp->currentMowingPath;
        currentMowingArea = cp->currentMowingArea;
        currentMowingPathIndex = cp->currentMowingPathIndex;
        currentMowingPlanDigest = cp->currentMowingPlanDigest;
        currentMowingAngleIncrementSum = cp->currentMowingAngleIncrementSum;
        found = true;
        break;
      }
    }
    bag.close();
  }
  return found;
}
