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

using namespace std::chrono_literals;

ActionObservedCostClient::ActionObservedCostClient(
  const std::string & node_name,
  const std::chrono::nanoseconds & rate)
: plansys2::ActionExecutorClient(node_name, rate)
{
  data_collection_pub_ = create_publisher<plansys2_msgs::msg::ActionExecutionDataCollection>(
    "/action_execution_data_collection", 10);

  problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>(get_name());
  domain_expert_ = std::make_shared<plansys2::DomainExpertClient>(get_name());


  auto fully_declare_parameter = [&](const std::string & param_name, auto default_val, auto type,
      const std::string & description) -> auto
    {
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.set__description(description);
      desc.type = type;
      declare_parameter(param_name, default_val, desc);
    };

  std::string default_path_updated_fluents = std::string("/tmp/") + get_namespace() +
    "/updated_fluents.txt";
  std::string default_path_updated_problem = std::string("/tmp/") + get_namespace() +
    "/updated_problem.pddl";

  fully_declare_parameter(
    "update_fluent", false, rclcpp::PARAMETER_BOOL,
    "Update fluent after action execution");
  fully_declare_parameter(
    "update_knowledge_base", false, rclcpp::PARAMETER_BOOL,
    "Update knowledge base after action execution");
  fully_declare_parameter(
    "save_updated_action_cost", false, rclcpp::PARAMETER_BOOL,
    "Save the action cost even if it is not updated");
  fully_declare_parameter(
    "updated_fluents_path", default_path_updated_fluents,
    rclcpp::PARAMETER_STRING,
    "Full path where save the file of updated action cost");
  fully_declare_parameter(
    "updated_problem_path", default_path_updated_problem,
    rclcpp::PARAMETER_STRING,
    "Full path where save the updated pddl problem");

  fully_declare_parameter("fluent_to_update", "", rclcpp::PARAMETER_STRING, "Fluent to update");
  fully_declare_parameter(
    "fluent_args",
    std::vector<int>(), rclcpp::PARAMETER_INTEGER_ARRAY, "Fluent arguments");

  get_parameter("save_updated_action_cost", save_updated_action_cost_);
  get_parameter("updated_fluents_path", updated_fluents_path_);
  get_parameter("updated_problem_path", updated_problem_path_);
  get_parameter("update_fluent", update_fluents_);
  get_parameter("update_knowledge_base", update_knowledge_base_);

  if (!is_valid_path(updated_fluents_path_)) {
    RCLCPP_WARN(get_logger(), "Invalid path %s", updated_fluents_path_.c_str());
    updated_fluents_path_ = default_path_updated_fluents;
  }
  if (!is_valid_path(updated_problem_path_)) {
    RCLCPP_WARN(get_logger(), "Invalid path %s", updated_problem_path_.c_str());
    updated_problem_path_ = default_path_updated_problem;
  }
  RCLCPP_INFO(get_logger(), "Path fluents: %s", updated_fluents_path_.c_str());
  RCLCPP_INFO(get_logger(), "Path problem: %s", updated_problem_path_.c_str());

  get_parameter("fluent_to_update", fluent_to_update_);
  get_parameter("fluent_args", fluent_args_);
  save_updated_action_cost_ = true;


  fully_declare_parameter(
    "state_observer_param_plugin", "state_observers::KalmanFilterParam",
    rclcpp::PARAMETER_STRING,
    "Plugin for the state observer parameters ROS 2 interface");
  fully_declare_parameter(
    "state_observer_plugin", "state_observers::KalmanFilter",
    rclcpp::PARAMETER_STRING,
    "Plugin for the state observer");

  std::string state_observer_param_plugin =
    get_parameter("state_observer_param_plugin").as_string();
  state_observer_plugin_name_ = get_parameter("state_observer_plugin").as_string();

  state_observer_params_loader_.reset(
    new pluginlib::ClassLoader<state_observer::StateObserverParam>(
      "state_observers",
      "state_observer::StateObserverParam"));
  state_observer_loader_.reset(
    new pluginlib::ClassLoader<state_observer::StateObserver>(
      "state_observers",
      "state_observer::StateObserver"));
  try {
    state_observer_params_ =
      state_observer_params_loader_->createSharedInstance(
      state_observer_param_plugin);
  } catch (pluginlib::PluginlibException & ex) {
    RCLCPP_ERROR(
      get_logger(), "The plugin failed to load for some reason. Error: %s", ex.what());
  }

  if (!state_observer_loader_->isClassAvailable(state_observer_plugin_name_)) {
    RCLCPP_ERROR(
      get_logger(), "State Observer Class: %s is not available",
      state_observer_plugin_name_.c_str());
    throw std::runtime_error("State Observer Class: %s is not available. Check the plugin name");
  }

  // Update knowledge base client
  async_internal_node_ = std::make_shared<rclcpp::Node>(
    "_async_internal_node_" + std::string(get_name()));

  async_internal_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  async_internal_executor_->add_node(async_internal_node_);
  if (update_knowledge_base_) {
    update_knowledge_base_client_ = async_internal_node_->create_client<AffectNodeInfo>(
      "/update_knowledge");
    if (!update_knowledge_base_client_->wait_for_service(1s)) {
      RCLCPP_WARN(
        get_logger(),
        "Service: update_knowledge is not available. KB will not be updated");
    }
  }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ActionObservedCostClient::on_configure(const rclcpp_lifecycle::State & state)
{
  auto return_value = ActionExecutorClient::on_configure(state);

  for (const auto & arg : specialized_arguments_) {
    declare_parameter<std::vector<std::string>>(arg, std::vector<std::string>());
    get_parameter(arg, associated_arguments_[arg]);
  }
  state_observer_params_->initialize(shared_from_this());
  return return_value;
}

std::string
ActionObservedCostClient::get_arguments_hash()
{
  auto action_arguments = get_arguments();
  std::string arguments_hash = std::accumulate(
    action_arguments.begin(), action_arguments.end(), std::string(),
    [](const std::string & a, const std::string & b) {
      return a.empty() ? b : a + "_" + b;
    }
  );
  return arguments_hash;
}

void
ActionObservedCostClient::finish(
  bool success,
  float completion,
  const std::string & status,
  const double measured_action_cost)
{
  RCLCPP_INFO(
    get_logger(), "Action finished: %s",
    get_action_name().c_str());
  RCLCPP_INFO(
    get_logger(), "Measured action cost %f", measured_action_cost);
  Eigen::VectorXd residual(1);
  if (!action_cost_) {
    residual << measured_action_cost;
  } else {
    RCLCPP_INFO(get_logger(), "Nominal action cost %f", action_cost_->nominal_cost);
    residual << measured_action_cost - action_cost_->nominal_cost;
    RCLCPP_INFO(get_logger(), "Residual: %f", residual[0]);
  }
  auto arguments_hash = get_arguments_hash();

  if (observed_action_cost_.find(arguments_hash) == observed_action_cost_.end()) {
    RCLCPP_WARN(get_logger(), "Observer not found for arguments hash %s", arguments_hash.c_str());
    observed_action_cost_[arguments_hash] = load_state_observer();
  } else {
    if (observed_action_cost_[arguments_hash]->is_initialized()) {
      observed_action_cost_[arguments_hash]->update(residual);
      RCLCPP_INFO(get_logger(), "Updated observer");
    } else {
      observed_action_cost_[arguments_hash]->initialize(residual);
      RCLCPP_INFO(get_logger(), "Observer initialized");
    }
    RCLCPP_INFO(get_logger(), "State %f", observed_action_cost_[arguments_hash]->get_state()[0]);
    RCLCPP_INFO(get_logger(), "Output %f", observed_action_cost_[arguments_hash]->get_output()[0]);
  }

  std::string updated_fluent = "";
  if (update_fluents_) {
    // check if arguments_hash is in observed_action_cost_
    if (observed_action_cost_.find(arguments_hash) == observed_action_cost_.end()) {
      RCLCPP_INFO(get_logger(), "Observer not found for arguments hash %s", arguments_hash.c_str());
    }
    //check dim of get_state()
    if (action_cost_) {
      RCLCPP_INFO(get_logger(), "Action cost %f", action_cost_->nominal_cost);
    } else {
      RCLCPP_INFO(get_logger(), "Action cost nullptr");
    }
    double updated_cost = observed_action_cost_[arguments_hash]->get_state()[0] +
      action_cost_->nominal_cost;
    std::string updated_fluent_string = update_fluent_string(updated_cost);
    RCLCPP_INFO(get_logger(), "Update fluent: %s", updated_fluent_string.c_str());
    plansys2::Function updated_fluent = plansys2::Function(updated_fluent_string);
    if (!problem_expert_->updateFunction(updated_fluent)) {
      RCLCPP_WARN(get_logger(), "Error updating fluent");
    }

    if (update_knowledge_base_) {
      auto update_knowledge_request = std::make_shared<AffectNodeInfo::Request>();
      update_knowledge_request->node = updated_fluent;
      try {
        update_knowledge_request->value =
          observed_action_cost_[arguments_hash]->get_state_variance()[0];
        update_knowledge_request->additional_info = "variance";
      } catch (const std::exception & e) {
        RCLCPP_INFO(get_logger(), "This state observer does not provide state variance");
      }
      auto future_result = update_knowledge_base_client_->async_send_request(
        update_knowledge_request);
      rclcpp::FutureReturnCode status = async_internal_executor_->spin_until_future_complete(
        future_result, std::chrono::seconds(5));
      if (status != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_WARN(
          get_logger(), "Error updating knowledge base: %s",
          update_knowledge_base_client_->get_service_name());
      }
      auto result = *future_result.get();

      if (!result.success) {
        RCLCPP_WARN_STREAM(
          get_logger(),
          update_knowledge_base_client_->get_service_name() << ": " <<
            result.error_info);
      }
    }
  }


  if (data_collection_ptr_) {
    // measured cost
    data_collection_ptr_->measured_action_cost.nominal_cost = measured_action_cost;
    // residual cost (observer state)
    data_collection_ptr_->residual_action_cost.nominal_cost =
      observed_action_cost_[arguments_hash]->get_state()[0];
    // std dev cost (observer variance), if available by the observer
    try {
      data_collection_ptr_->residual_action_cost.std_dev_cost =
        std::sqrt(observed_action_cost_[arguments_hash]->get_state_variance()[0]);
      RCLCPP_INFO(
        get_logger(), "Variance %f", observed_action_cost_[arguments_hash]->get_state_variance());
    } catch (const std::exception & e) {
      RCLCPP_INFO(get_logger(), "This state observer does not provide state variance");
    }
    data_collection_ptr_->t_start = ActionExecutorClient::get_start_time();
    data_collection_ptr_->t_end = now();
    data_collection_pub_->publish(*data_collection_ptr_);
  }

  if (save_updated_action_cost_) {
    RCLCPP_INFO(get_logger(), "Save updated action cost: %s", updated_fluent.c_str());
    // Save updated problem to the file
    std::string problem = problem_expert_->getProblem();
    save_updated_problem(problem);

    RCLCPP_INFO(get_logger(), "Save updated fluent: %s", updated_fluent.c_str());
    // Save updated fluents to the file
    save_updated_fluent(updated_fluent);
  }
  ActionExecutorClient::finish(success, completion, status);
  RCLCPP_INFO(get_logger(), "Finish");
}

void
ActionObservedCostClient::send_response(
  const plansys2_msgs::msg::ActionExecution::SharedPtr msg)
{
  RCLCPP_INFO(
    get_logger(), "Action %s send response computed nominal cost: %f",
    get_action_name().c_str(), action_cost_->nominal_cost);

  plansys2_msgs::msg::ActionExecution msg_resp(*msg);
  msg_resp.type = plansys2_msgs::msg::ActionExecution::RESPONSE;
  msg_resp.node_id = get_name();

  // set action cost as the theoretical if there is not the observed one
  auto arguments_hash = get_arguments_hash();
  if (observed_action_cost_.find(arguments_hash) != observed_action_cost_.end()) {
    msg_resp.action_cost = plansys2_msgs::msg::ActionCost();
    msg_resp.action_cost.nominal_cost = action_cost_->nominal_cost +
      observed_action_cost_[arguments_hash]->get_state()[0];
    try {
      msg_resp.action_cost.std_dev_cost =
        std::sqrt(observed_action_cost_[arguments_hash]->get_state_variance()[0]);
    } catch (const std::exception & e) {
      msg_resp.action_cost.std_dev_cost = 0.0;
      RCLCPP_INFO(get_logger(), "This state observer does not provide state variance");
    }
  } else {

    observed_action_cost_[arguments_hash] = load_state_observer();

    msg_resp.action_cost = *action_cost_;
    try {
      msg_resp.action_cost.std_dev_cost =
        std::sqrt(observed_action_cost_[arguments_hash]->get_state_variance()[0]);
    } catch (const std::exception & e) {
      msg_resp.action_cost.std_dev_cost = 0.0;
      RCLCPP_INFO(get_logger(), "This state observer does not provide state variance");
    }
  }
  action_hub_pub_->publish(msg_resp);

  data_collection_ptr_ = std::make_shared<plansys2_msgs::msg::ActionExecutionDataCollection>();
  data_collection_ptr_->action_execution = msg_resp;
  // nominal
  data_collection_ptr_->nominal_action_cost = *action_cost_;
  // estimated
  data_collection_ptr_->estimated_action_cost.nominal_cost = msg_resp.action_cost.nominal_cost;
  data_collection_ptr_->estimated_action_cost.std_dev_cost = msg_resp.action_cost.std_dev_cost;

}

std::string
ActionObservedCostClient::update_fluent_string(const double & fluent_value)
{
  auto args = get_arguments();

  std::string fluent_string = "(= (" + fluent_to_update_ + " ";

  for (const auto & arg : fluent_args_) {
    if (arg < args.size()) {
      fluent_string += args[arg] + " ";
    } else {
      RCLCPP_INFO(get_logger(), "Error: arg %d not found in args", arg);
    }
  }
  fluent_string += ") " + std::to_string(fluent_value) + ")";
  return fluent_string;
}

std::string
ActionObservedCostClient::update_fluent(const double & fluent_value)
{
  auto args = get_arguments();

  std::string fluent_string = "(= (" + fluent_to_update_ + " ";

  for (const auto & arg : fluent_args_) {
    if (arg < args.size()) {
      fluent_string += args[arg] + " ";
    } else {
      RCLCPP_INFO(get_logger(), "Error: arg %d not found in args", arg);
    }
  }
  fluent_string += ") " + std::to_string(fluent_value) + ")";
  RCLCPP_INFO(get_logger(), "Update fluent: %s", fluent_string.c_str());
  problem_expert_->updateFunction(plansys2::Function(fluent_string));
  return fluent_string;
}

bool
ActionObservedCostClient::is_valid_path(const std::string & path_str)
{
  std::filesystem::path path(path_str);
  return std::filesystem::exists(path);
}

void
ActionObservedCostClient::save_updated_fluent(const std::string & updated_fluent)
{
  RCLCPP_INFO(get_logger(), "Save updated fluent: %s", updated_fluent.c_str());
  if (updated_fluent == "") {
    return;
  }
  RCLCPP_INFO(get_logger(), "New fluent: %s", updated_fluent.c_str());

  std::regex re(R"(\(= \(([^)]+)\))");
  std::smatch match;
  std::string search_string;

  if (std::regex_search(updated_fluent, match, re) && match.size() > 1) {
    search_string = match.str(0); // First match is (= (fluent args) value), second fluent args
  } else {
    RCLCPP_WARN(get_logger(), "Warning: invalid updated_fluent format. No match found.");
    return;
  }
  RCLCPP_INFO(get_logger(), "Search string: %s", search_string.c_str());

  std::ifstream infile(updated_fluents_path_);
  std::vector<std::string> lines;
  std::string line;
  bool found = false;

  // If the file exists read all the lines
  if (infile.is_open()) {
    while (std::getline(infile, line)) {
      // Check if the line contains the substring
      if (line.find(search_string) != std::string::npos) {
        RCLCPP_INFO(get_logger(), "Found in file: %s", line.c_str());
        line = updated_fluent;        // Replace with the new fluent
        found = true;
      }
      lines.push_back(line);
    }
    infile.close();
  }

  // If the fluent is not found in the file, add it
  if (!found) {
    lines.push_back(updated_fluent);
  }

  // Open the file, if it does not exist, create it.
  std::ofstream outfile(updated_fluents_path_);
  if (!outfile) {
    RCLCPP_WARN(
      get_logger(), "Warning: impossible to open file: %s to write updated fluents.",
      updated_fluents_path_.c_str());
    return;
  }

  for (const auto & l : lines) {
    outfile << l << std::endl;
  }
  outfile.close();
  RCLCPP_INFO(get_logger(), "Fluents updated!");
}

void
ActionObservedCostClient::save_updated_problem(const std::string & updated_problem)
{
  RCLCPP_INFO(get_logger(), "Updated problem: %s", updated_problem.c_str());

  // Now open the file for writing (this will overwrite the existing file)
  std::ofstream outfile_problem(updated_problem_path_);
  if (!outfile_problem) {
    RCLCPP_WARN(
      get_logger(), "Warning: impossible to open file: %s to write updated problem.",
      updated_problem_path_.c_str());
    return;
  }
  // Write the updated problem to the file
  outfile_problem << updated_problem << std::endl;

  // Close the file
  outfile_problem.close();
}

bool
ActionObservedCostClient::should_execute(
  const std::string & action, const std::vector<std::string> & args)
{
  if (action != action_managed_) {
    return false;
  }

  if (!specialized_arguments_.empty()) {
    for (const auto & specialized_arg : specialized_arguments_) {
      size_t pos = specialized_arg.find("?");
      if (pos != std::string::npos) {
        int arg_num = std::stoi(specialized_arg.substr(pos + 1));
        if (arg_num < args.size()) {
          if (std::find(
              associated_arguments_[specialized_arg].begin(),
              associated_arguments_[specialized_arg].end(),
              args[0].c_str()) == associated_arguments_[specialized_arg].end())
          {
            RCLCPP_INFO(
              get_logger(), "Instance: %s, for action: %s am not specialized for this args",
              get_name(), get_action_name().c_str());
            return false;
          }
        } else {
          RCLCPP_INFO(
            get_logger(), "You specified arg n: %d but the action has n: %d args", arg_num,
            args.size());
        }
      } else {
        RCLCPP_INFO(get_logger(), "No ? in specialized args");
      }
    }
  }
  RCLCPP_INFO(
    get_logger(), "I [%s, %s] am specialized for this args",
    get_name(), get_action_name().c_str());
  return true;
}

std::shared_ptr<state_observer::StateObserver>
ActionObservedCostClient::load_state_observer()
{
  std::shared_ptr<state_observer::StateObserver> state_observer;
  try {
    state_observer =
      state_observer_loader_->createSharedInstance(
      state_observer_plugin_name_);
  } catch (pluginlib::PluginlibException & ex) {
    RCLCPP_ERROR(
      get_logger(), "The plugin: %s, failed to load for some reason. Error: %s", state_observer_plugin_name_,
      ex.what());
    throw std::runtime_error("The plugin failed to load for some reason.");
  }

  try {
    state_observer->set_parameters(
      state_observer_params_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Exception caught in state observer set_parameters: %s", e.what());
    throw std::runtime_error("The plugin failed to set parameters.");
  }

  return state_observer;
}

}  // namespace plansys2_actions_clients
