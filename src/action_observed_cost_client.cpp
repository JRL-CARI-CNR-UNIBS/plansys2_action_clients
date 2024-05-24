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

#include "plansys2_action_clients/action_observed_cost_client.hpp"

namespace plansys2_actions_clients
{

ActionObservedCostClient::ActionObservedCostClient(
  const std::string & node_name,
  const std::chrono::nanoseconds & rate)
  : plansys2::ActionExecutorClient(node_name, rate){
    data_collection_pub_ = create_publisher<plansys2_msgs::msg::ActionExecutionDataCollection>(
      "/action_execution_data_collection", 10);
  }

std::string
ActionObservedCostClient::get_arguments_hash()
{
  auto action_arguments = get_arguments();
  std::string arguments_hash = std::accumulate(action_arguments.begin(), action_arguments.end(), std::string(),
      [](const std::string& a, const std::string& b) {
          return a.empty() ? b : a + "_" + b;
      }
  );
  return arguments_hash;
}

void
ActionObservedCostClient::finish(bool success, 
                                 float completion, 
                                 const std::string & status,
                                 const double measured_action_cost)
{
  RCLCPP_INFO(get_logger(), "ActionObservedCostClient::finish");
  RCLCPP_INFO(get_logger(), "Measured action cost %f", measured_action_cost);
  Eigen::VectorXd residual(1);
  if(!action_cost_) {
    residual << measured_action_cost;  
  }
  else {
    RCLCPP_INFO(get_logger(), "Nominal action cost %f", action_cost_->nominal_cost);
    residual << measured_action_cost - action_cost_->nominal_cost;
    RCLCPP_INFO(get_logger(), "Residual %f", residual[0]);
  }
  
  auto arguments_hash = get_arguments_hash();
  
  if(observed_action_cost_.find(arguments_hash) == observed_action_cost_.end()) {

    Eigen::MatrixXd A = Eigen::MatrixXd::Constant(1, 1, 1);
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(1, 1);
    Eigen::MatrixXd C = Eigen::MatrixXd::Constant(1, 1, 1);
    Eigen::MatrixXd L = Eigen::MatrixXd::Identity(1, 1) * 0.6;
    std::shared_ptr<state_observer::Luenberger> luenberger_ptr;
    
    try {
        luenberger_ptr = std::make_shared<state_observer::Luenberger>(A, B, C, L);
    } catch (const std::invalid_argument& e) {
        std::cerr << "Exception caught: " << e.what() << std::endl;
    }

    observed_action_cost_[arguments_hash] = luenberger_ptr;
    observed_action_cost_[arguments_hash]->initialize(residual);
    RCLCPP_INFO(get_logger(), "Initialized observer");
    RCLCPP_INFO(get_logger(), "First state %f", luenberger_ptr->get_state()[0]);
  }
  else {
    observed_action_cost_[arguments_hash]->update(residual);
    RCLCPP_INFO(get_logger(), "Updated observer");
    RCLCPP_INFO(get_logger(), "State %f", observed_action_cost_[arguments_hash]->get_state()[0]);
    // RCLCPP_INFO(get_logger(), "Variance %f", observed_action_cost_[arguments_hash]->get_state_variance());
    RCLCPP_INFO(get_logger(), "Output %f", observed_action_cost_[arguments_hash]->get_output()[0]);
  }
  if(data_collection_ptr_) {
    data_collection_ptr_->measured_action_cost.nominal_cost = measured_action_cost;
    
    data_collection_ptr_->residual_action_cost.nominal_cost = observed_action_cost_[arguments_hash]->get_state()[0];
    // data_collection_ptr_->residual_action_cost.std_dev_cost = observed_action_cost_[arguments_hash]->()[0];; // TODO: observed_action_cost_[arguments_hash]->get_state_variance();
    data_collection_pub_->publish(*data_collection_ptr_);
    RCLCPP_INFO(get_logger(), "Published data collection");
  }
  ActionExecutorClient::finish(success, completion, status);
}

// void 
// ActionObservedCostClient::set_action_cost(const ActionCostPtr & action_cost,
//                                           plansys2_msgs::msg::ActionExecution::SharedPtr msg)
// {
//   action_cost_ = action_cost;
//   send_response(msg);
// }

void
ActionObservedCostClient::send_response(
  const plansys2_msgs::msg::ActionExecution::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "ActionObservedCostClient::send_response");
  RCLCPP_INFO(get_logger(), "NOMINAL ACTION COST: %f", action_cost_->nominal_cost);
  plansys2_msgs::msg::ActionExecution msg_resp(*msg);
  msg_resp.type = plansys2_msgs::msg::ActionExecution::RESPONSE;
  msg_resp.node_id = get_name();
  
  // set action cost as the theoretical if there is not the observed one
  auto arguments_hash = get_arguments_hash();
  if(observed_action_cost_.find(arguments_hash) != observed_action_cost_.end()) 
  {
    msg_resp.action_cost = plansys2_msgs::msg::ActionCost();
    msg_resp.action_cost.nominal_cost = action_cost_->nominal_cost +
                                        observed_action_cost_[arguments_hash]->get_state()[0];
    msg_resp.action_cost.std_dev_cost = 0.0; // TODO: observed_action_cost_[arguments_hash]->get_state_variance()
  }
  else {
    msg_resp.action_cost = *action_cost_;
  }
  action_hub_pub_->publish(msg_resp);
  RCLCPP_INFO(get_logger(), "Preparing data collection");
  data_collection_ptr_ = std::make_shared<plansys2_msgs::msg::ActionExecutionDataCollection>();
  data_collection_ptr_ -> action_execution = msg_resp;

  data_collection_ptr_ -> nominal_action_cost = *action_cost_;
  data_collection_ptr_ -> estimated_action_cost.nominal_cost = msg_resp.action_cost.nominal_cost;
  data_collection_ptr_ -> estimated_action_cost.std_dev_cost = msg_resp.action_cost.std_dev_cost;
  // plansys2::msg::ActionExecutionDataCollection data_collection;
  // data_collection.action_execution = *msg_resp;

}

}  // namespace plansys2_actions_clients

