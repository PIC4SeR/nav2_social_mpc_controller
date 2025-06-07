// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_SOCIAL_MPC_CONTROLLER__CRITIC_FUNCTION_HPP_
#define NAV2_SOCIAL_MPC_CONTROLLER__CRITIC_FUNCTION_HPP_

#include <string>
#include <memory>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

// #include "nav2_mppi_controller/tools/parameters_handler.hpp"
// #include "nav2_mppi_controller/critic_data.hpp"

namespace nav2_social_mpc_controller::critics
{

/**
 * @class mppi::critics::CriticFunction
 * @brief Abstract critic objective function to score trajectories
 */
class CriticFunction
{
public:
  /**
   * @brief Constructor for mppi::critics::CriticFunction
   */
  CriticFunction() = default;

  /**
   * @brief Destructor for mppi::critics::CriticFunction
   */
  virtual ~CriticFunction() = default;

  /**
   * @brief Configure critic on bringup
   * @param parent WeakPtr to node
   * @param parent_name name of the controller
   * @param name Name of plugin
   * @param costmap_ros Costmap2DROS object of environment
   * @param dynamic_parameter_handler Parameter handler object
   */
  void on_configure(rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string& parent_name,
                    const std::string& name, ParametersHandler* param_handler)
  {
    parent_ = parent;
    logger_ = parent_.lock()->get_logger();
    name_ = name;
    parent_name_ = parent_name;
    parameters_handler_ = param_handler;

    auto getParam = parameters_handler_->getParamGetter(name_);
    getParam(enabled_, "enabled", true);

    initialize();
  }

  /**
   * @brief Create a Ceres cost function object for the critic
   * @return ceres::CostFunction* Pointer to the created cost function
   */
  virtual ceres::CostFunction* create() const = 0;

  /**
   * @brief Initialize critic
   */
  virtual void initialize() = 0;

  /**
   * @brief Get name of critic
   */
  std::string getName()
  {
    return name_;
  }

protected:
  bool enabled_;
  std::string name_, parent_name_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;

  ParametersHandler* parameters_handler_;
  rclcpp::Logger logger_{ rclcpp::get_logger("SocialMPCController") };
};

}  // namespace nav2_social_mpc_controller::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITIC_FUNCTION_HPP_
