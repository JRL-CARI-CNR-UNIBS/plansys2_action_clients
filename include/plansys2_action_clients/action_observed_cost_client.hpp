// Copyright 2024 National Council of Research of Italy (CNR) - Intelligent Robotics Lab
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

#ifndef PLANSYS2_ACTIONS_CLIENTS__ACTION_OBSERVER_COST_CLIENT_HPP_
#define PLANSYS2_ACTIONS_CLIENTS__ACTION_OBSERVER_COST_CLIENT_HPP_

#include <memory>

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "plansys2_msgs/msg/action_execution_data_collection.hpp"
#include "state_observers/luenberger.hpp"

namespace plansys2_actions_clients
{

class ActionObservedCostClient : public plansys2::ActionExecutorClient
{
    
public:
  ActionObservedCostClient(
  const std::string & node_name,
  const std::chrono::nanoseconds & rate);

  // void finish(bool success, float completion, const std::string & status) override;
  void finish(bool success, float completion, const std::string & status, const double measured_action_cost);

  // void set_action_cost(const ActionCostPtr & action_cost,
  //                       const plansys2_msgs::msg::ActionExecution::SharedPtr msg);
        
protected:
  std::map<std::string, std::shared_ptr<state_observer::Luenberger>> observed_action_cost_;
  void send_response(const plansys2_msgs::msg::ActionExecution::SharedPtr msg) override;
  std::string get_arguments_hash();

private:
  using ActionExecutionDataCollection = plansys2_msgs::msg::ActionExecutionDataCollection;
  using ActionExecutionDataCollectionPtr = std::shared_ptr<ActionExecutionDataCollection>;
  ActionExecutionDataCollectionPtr data_collection_ptr_;
  rclcpp::Publisher<ActionExecutionDataCollection>::SharedPtr data_collection_pub_;
  // plansys2::msg::ActionExecutionDataCollectionPtr data_collection_;
};

}  // namespace plansys2_actions_clients

#endif  // PLANSYS2_ACTIONS_CLIENTS__ACTION_OBSERVER_COST_CLIENT_HPP_
