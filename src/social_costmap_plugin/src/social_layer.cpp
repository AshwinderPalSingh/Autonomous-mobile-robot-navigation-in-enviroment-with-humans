// Copyright 2026
// SPDX-License-Identifier: Apache-2.0

#include "social_costmap_plugin/social_layer.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "pluginlib/class_list_macros.hpp"

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace social_costmap_plugin
{

void SocialLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"SocialLayer: failed to lock lifecycle node"};
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("topic", rclcpp::ParameterValue(std::string("/pedestrians")));
  declareParameter("amplitude", rclcpp::ParameterValue(220.0));
  declareParameter("sigma_base", rclcpp::ParameterValue(0.45));
  declareParameter("sigma_side_ratio", rclcpp::ParameterValue(0.6));
  declareParameter("speed_factor", rclcpp::ParameterValue(1.6));
  declareParameter("lethal_radius", rclcpp::ParameterValue(0.30));
  declareParameter("cutoff", rclcpp::ParameterValue(15.0));
  declareParameter("keep_time", rclcpp::ParameterValue(1.0));
  declareParameter("min_moving_speed", rclcpp::ParameterValue(0.05));

  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".topic", topic_);
  node->get_parameter(name_ + ".amplitude", amplitude_);
  node->get_parameter(name_ + ".sigma_base", sigma_base_);
  node->get_parameter(name_ + ".sigma_side_ratio", sigma_side_ratio_);
  node->get_parameter(name_ + ".speed_factor", speed_factor_);
  node->get_parameter(name_ + ".lethal_radius", lethal_radius_);
  node->get_parameter(name_ + ".cutoff", cutoff_);
  node->get_parameter(name_ + ".keep_time", keep_time_);
  node->get_parameter(name_ + ".min_moving_speed", min_moving_speed_);

  // Never paint LETHAL via the Gaussian itself; the lethal core handles that.
  amplitude_ = std::min(
    amplitude_, static_cast<double>(INSCRIBED_INFLATED_OBSTACLE));

  sub_ = node->create_subscription<social_nav_msgs::msg::Pedestrians>(
    topic_, rclcpp::SensorDataQoS(),
    std::bind(&SocialLayer::pedestriansCallback, this, std::placeholders::_1));

  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&SocialLayer::dynamicParametersCallback, this, std::placeholders::_1));

  current_ = true;

  RCLCPP_INFO(
    logger_,
    "SocialLayer '%s' initialized. topic=%s amplitude=%.0f sigma_base=%.2f "
    "speed_factor=%.2f lethal_radius=%.2f",
    name_.c_str(), topic_.c_str(), amplitude_, sigma_base_,
    speed_factor_, lethal_radius_);
}

void SocialLayer::pedestriansCallback(social_nav_msgs::msg::Pedestrians::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_msg_ = msg;
}

double SocialLayer::affectedRadius(double speed) const
{
  const double amp = std::max(amplitude_, cutoff_ + 1.0);
  const double sigma_max = sigma_base_ * (1.0 + speed_factor_ * speed);
  const double r = sigma_max * std::sqrt(2.0 * std::log(amp / cutoff_));
  return std::max(r, lethal_radius_) + 0.10;  // small safety margin
}

double SocialLayer::gaussianCost(
  double dx, double dy, double heading, double speed) const
{
  double sigma_front = sigma_base_;
  double sigma_back = sigma_base_;
  double sigma_side = sigma_base_;

  double lx = dx;
  double ly = dy;

  if (speed >= min_moving_speed_) {
    // Rotate the offset into the pedestrian's velocity-aligned frame.
    const double ca = std::cos(heading);
    const double sa = std::sin(heading);
    lx = ca * dx + sa * dy;    // along direction of travel
    ly = -sa * dx + ca * dy;   // lateral

    sigma_front = sigma_base_ * (1.0 + speed_factor_ * speed);
    sigma_side = sigma_base_ * sigma_side_ratio_;
  }
  // Standing person: isotropic Gaussian with sigma_base (heading undefined).

  const double sx = (lx >= 0.0) ? sigma_front : sigma_back;
  const double f = std::exp(
    -0.5 * ((lx * lx) / (sx * sx) + (ly * ly) / (sigma_side * sigma_side)));

  return amplitude_ * f;
}

void SocialLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  // This layer must always report current, even when empty or disabled,
  // otherwise Nav2 will consider the whole costmap stale.
  current_ = true;

  if (!enabled_) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  pedestrians_.clear();

  if (!latest_msg_ || latest_msg_->pedestrians.empty()) {
    return;
  }

  // Staleness check: drop old detections instead of freezing them in place.
  const rclcpp::Time now = clock_->now();
  const rclcpp::Time stamp(latest_msg_->header.stamp);
  if (stamp.nanoseconds() > 0 && (now - stamp).seconds() > keep_time_) {
    latest_msg_.reset();
    return;
  }

  const std::string global_frame = layered_costmap_->getGlobalFrameID();
  geometry_msgs::msg::TransformStamped tf_to_global;
  try {
    tf_to_global = tf_->lookupTransform(
      global_frame, latest_msg_->header.frame_id, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "SocialLayer: cannot transform %s -> %s: %s",
      latest_msg_->header.frame_id.c_str(), global_frame.c_str(), ex.what());
    return;
  }

  for (const auto & ped : latest_msg_->pedestrians) {
    geometry_msgs::msg::PointStamped p_in, p_out;
    p_in.header = latest_msg_->header;
    p_in.point = ped.position;
    tf2::doTransform(p_in, p_out, tf_to_global);

    // Velocity is a free vector: doTransform on Vector3 applies rotation only.
    geometry_msgs::msg::Vector3Stamped v_in, v_out;
    v_in.header = latest_msg_->header;
    v_in.vector = ped.velocity;
    tf2::doTransform(v_in, v_out, tf_to_global);

    TrackedPedestrian tp;
    tp.x = p_out.point.x;
    tp.y = p_out.point.y;
    tp.vx = v_out.vector.x;
    tp.vy = v_out.vector.y;
    pedestrians_.push_back(tp);

    const double r = affectedRadius(std::hypot(tp.vx, tp.vy));
    *min_x = std::min(*min_x, tp.x - r);
    *min_y = std::min(*min_y, tp.y - r);
    *max_x = std::max(*max_x, tp.x + r);
    *max_y = std::max(*max_y, tp.y + r);
  }
}

void SocialLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (pedestrians_.empty()) {
    return;
  }

  for (const auto & ped : pedestrians_) {
    const double speed = std::hypot(ped.vx, ped.vy);
    const double heading = std::atan2(ped.vy, ped.vx);
    const double r = affectedRadius(speed);

    // Cell-space bounding box for this pedestrian, clipped to the update window.
    int start_i, start_j, end_i, end_j;
    master_grid.worldToMapEnforceBounds(ped.x - r, ped.y - r, start_i, start_j);
    master_grid.worldToMapEnforceBounds(ped.x + r, ped.y + r, end_i, end_j);
    start_i = std::max(start_i, min_i);
    start_j = std::max(start_j, min_j);
    end_i = std::min(end_i, max_i - 1);
    end_j = std::min(end_j, max_j - 1);

    for (int j = start_j; j <= end_j; ++j) {
      for (int i = start_i; i <= end_i; ++i) {
        double wx, wy;
        master_grid.mapToWorld(i, j, wx, wy);
        const double dx = wx - ped.x;
        const double dy = wy - ped.y;

        unsigned char new_cost;
        if (std::hypot(dx, dy) <= lethal_radius_) {
          new_cost = LETHAL_OBSTACLE;
        } else {
          const double c = gaussianCost(dx, dy, heading, speed);
          if (c < cutoff_) {
            continue;
          }
          new_cost = static_cast<unsigned char>(
            std::min(c, static_cast<double>(INSCRIBED_INFLATED_OBSTACLE)));
        }

        const unsigned char old_cost = master_grid.getCost(i, j);
        if (old_cost == NO_INFORMATION) {
          master_grid.setCost(i, j, new_cost);
        } else {
          master_grid.setCost(i, j, std::max(old_cost, new_cost));
        }
      }
    }
  }
}

void SocialLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_msg_.reset();
  pedestrians_.clear();
  current_ = true;
}

rcl_interfaces::msg::SetParametersResult SocialLayer::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<std::mutex> lock(mutex_);
  rcl_interfaces::msg::SetParametersResult result;

  for (const auto & param : parameters) {
    const auto & param_name = param.get_name();
    if (param_name == name_ + ".enabled") {
      enabled_ = param.as_bool();
    } else if (param_name == name_ + ".amplitude") {
      amplitude_ = std::min(
        param.as_double(), static_cast<double>(INSCRIBED_INFLATED_OBSTACLE));
    } else if (param_name == name_ + ".sigma_base") {
      sigma_base_ = std::max(0.05, param.as_double());
    } else if (param_name == name_ + ".sigma_side_ratio") {
      sigma_side_ratio_ = std::max(0.05, param.as_double());
    } else if (param_name == name_ + ".speed_factor") {
      speed_factor_ = std::max(0.0, param.as_double());
    } else if (param_name == name_ + ".lethal_radius") {
      lethal_radius_ = std::max(0.0, param.as_double());
    } else if (param_name == name_ + ".cutoff") {
      cutoff_ = std::max(1.0, param.as_double());
    } else if (param_name == name_ + ".keep_time") {
      keep_time_ = std::max(0.1, param.as_double());
    } else if (param_name == name_ + ".min_moving_speed") {
      min_moving_speed_ = std::max(0.0, param.as_double());
    }
  }

  result.successful = true;
  return result;
}

}  // namespace social_costmap_plugin

// Register with pluginlib so Nav2 costmaps can load this layer by name.
PLUGINLIB_EXPORT_CLASS(social_costmap_plugin::SocialLayer, nav2_costmap_2d::Layer)
