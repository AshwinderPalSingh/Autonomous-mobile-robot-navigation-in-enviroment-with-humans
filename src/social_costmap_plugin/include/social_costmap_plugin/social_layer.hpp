// Copyright 2026
// SPDX-License-Identifier: Apache-2.0

#ifndef SOCIAL_COSTMAP_PLUGIN__SOCIAL_LAYER_HPP_
#define SOCIAL_COSTMAP_PLUGIN__SOCIAL_LAYER_HPP_

#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "social_nav_msgs/msg/pedestrians.hpp"

namespace social_costmap_plugin
{

/// A pedestrian already transformed into the costmap's global frame.
struct TrackedPedestrian
{
  double x{0.0};
  double y{0.0};
  double vx{0.0};
  double vy{0.0};
};

/**
 * @class SocialLayer
 * @brief Nav2 costmap layer that paints a velocity-scaled, asymmetric 2D
 *        Gaussian cost field around each pedestrian, plus a lethal core
 *        over the body itself.
 *
 * Cost model (Kirby-style proxemics, the classical social-navigation baseline):
 *
 *   sigma_front = sigma_base * (1 + speed_factor * |v|)   // ahead of person
 *   sigma_back  = sigma_base                              // behind person
 *   sigma_side  = sigma_base * sigma_side_ratio           // lateral
 *
 *   cost(dx, dy) = amplitude * exp( -0.5 * ( lx^2/sx^2 + ly^2/sside^2 ) )
 *
 * where (lx, ly) is the cell offset rotated into the pedestrian's velocity
 * frame and sx is sigma_front for lx >= 0, sigma_back otherwise. Cells within
 * lethal_radius of the body are set to LETHAL_OBSTACLE. Costs are merged into
 * the master grid with a max() policy, so this layer only ever raises cost.
 *
 * Detections arrive on a social_nav_msgs/Pedestrians topic in any TF frame;
 * the layer transforms them into the costmap global frame each update cycle.
 */
class SocialLayer : public nav2_costmap_2d::Layer
{
public:
  SocialLayer() = default;
  ~SocialLayer() override = default;

  // Layer interface -----------------------------------------------------
  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;
  void reset() override;
  bool isClearable() override {return false;}

private:
  void pedestriansCallback(social_nav_msgs::msg::Pedestrians::SharedPtr msg);

  /// Gaussian cost contribution at offset (dx, dy) from a pedestrian
  /// moving with the given heading [rad] and speed [m/s]. Range [0, amplitude].
  double gaussianCost(double dx, double dy, double heading, double speed) const;

  /// Radius beyond which the Gaussian falls below `cutoff` for this speed.
  double affectedRadius(double speed) const;

  /// Live parameter updates (lets you tune the layer mid-demo).
  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
    std::vector<rclcpp::Parameter> parameters);

  // Data ----------------------------------------------------------------
  rclcpp::Subscription<social_nav_msgs::msg::Pedestrians>::SharedPtr sub_;
  social_nav_msgs::msg::Pedestrians::SharedPtr latest_msg_;
  std::vector<TrackedPedestrian> pedestrians_;
  std::mutex mutex_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

  // Parameters ------------------------------------------------------------
  std::string topic_{"/pedestrians"};
  double amplitude_{220.0};        // peak cost, clamped below INSCRIBED (253)
  double sigma_base_{0.45};        // base stddev [m]
  double sigma_side_ratio_{0.6};   // lateral sigma = base * ratio
  double speed_factor_{1.6};       // front sigma growth per m/s
  double lethal_radius_{0.30};     // hard no-go radius over the body [m]
  double cutoff_{15.0};            // ignore contributions below this cost
  double keep_time_{1.0};          // drop detections older than this [s]
  double min_moving_speed_{0.05};  // below this, treat as standing (isotropic)
};

}  // namespace social_costmap_plugin

#endif  // SOCIAL_COSTMAP_PLUGIN__SOCIAL_LAYER_HPP_
