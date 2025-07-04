// Copyright (c) 2022 SRL -Service Robotics Lab, Pablo de Olavide University
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

#include "nav2_social_mpc_controller/critics/velocity_feasibility_cost_function.hpp"

namespace nav2_social_mpc_controller
{

VelocityFeasibilityCost::VelocityFeasibilityCost(
  double weight, unsigned int current_position, unsigned int control_horizon)
: weight_(weight)
{
  control_horizon_ = control_horizon;
  current_position_ = current_position;
}

}  // namespace nav2_social_mpc_controller