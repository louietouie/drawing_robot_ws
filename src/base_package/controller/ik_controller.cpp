// LOUIS: ROS2 Forward Command Controller

// Copyright 2021 Stogl Robotics Consulting UG (haftungsbescrhänkt)
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

#include "include/ik_controller.hpp"

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

namespace base_package
{
IKController::IKController()
: controller_interface::ControllerInterface(),
  rt_command_ptr_(nullptr),
  joints_command_subscriber_(nullptr),
  calculator_()
{
  // MultibodyPlant<double> _plant(0.0);
  // MultibodyPlant<double> acrobot(0.0);
  // Parser parser(&acrobot);
  // MultibodyPlant<double> acrobot(0.0)
  // acrobot.Finalize();
  // acrobot.CreateDefaultContext();
}

controller_interface::CallbackReturn IKController::on_init()
{
  
  RCLCPP_INFO(get_node()->get_logger(), "Initialization Started");

  try
  {
    declare_parameters();
    const std::string& urdf = get_robot_description();
    calculator_.load_model(urdf);    
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    RCLCPP_INFO(get_node()->get_logger(), "Custom Error");
    RCLCPP_INFO(get_node()->get_logger(), e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  
  //   calculator_.init(urdf);
  // if (!urdf.empty())
  // {
  //   drake::DifferentialInverseKinematicsCalculator calculator_;
  //   calculator_.init(urdf);
  // }
  // else
  // {
  //   // empty URDF is used for some tests
  //   RCLCPP_DEBUG(get_node()->get_logger(), "No URDF file given");
  // }

  return controller_interface::CallbackReturn::SUCCESS;

}

controller_interface::CallbackReturn IKController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto ret = this->read_parameters();
  if (ret != controller_interface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  joints_command_subscriber_ = get_node()->create_subscription<CmdType>(
    "~/commands", rclcpp::SystemDefaultsQoS(),
    [this](const CmdType::SharedPtr msg) { rt_command_ptr_.writeFromNonRT(msg); });

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration IKController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names = command_interface_types_; // Louis: command_interface_types_ is populated during read_parameters(), which is called in the lifecycle method on_configure()

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration IKController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interfaces_config.names = state_interface_types_; // Louis: command_interface_types_ is populated during read_parameters(), which is called in the lifecycle method on_configure()

  return state_interfaces_config;
}

controller_interface::CallbackReturn IKController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  //  check if we have all resources defined in the "points" parameter
  //  also verify that we *only* have the resources defined in the "points" parameter
  // ATTENTION(destogl): Shouldn't we use ordered interface all the time?
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    ordered_interfaces;
  if (
    !controller_interface::get_ordered_interfaces(
      command_interfaces_, command_interface_types_, std::string(""), ordered_interfaces) ||
    command_interface_types_.size() != ordered_interfaces.size())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %zu command interfaces, got %zu",
      command_interface_types_.size(), ordered_interfaces.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  // reset command buffer if a command came through callback when controller was inactive
  rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);

  RCLCPP_INFO(get_node()->get_logger(), "activate successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn IKController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // reset command buffer
  rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type IKController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  auto joint_commands = rt_command_ptr_.readFromRT();

  // const double shoulder_joint_position = state_interfaces_[0].get_value();
  // const double elbow_joint_position = state_interfaces_[1].get_value();

  // Eigen::Vector2d currentpos;
  // currentpos << shoulder_joint_position, elbow_joint_position;

  // Eigen::Vector2d goalpos;
  // goalpos << (*joint_commands)->data[0], (*joint_commands)->data[1];

  // // Eigen::VectorXd currentpos(shoulder_joint_position, elbow_joint_position);
  // // Eigen::VectorXd goalpos((*joint_commands)->data[0], (*joint_commands)->data[1]);
  // Eigen::VectorXd targetPositions = calculator_.calculateOneStep(goalpos, currentpos);

  // no command received yet
  if (!joint_commands || !(*joint_commands))
  {
    return controller_interface::return_type::OK;
  }

  if ((*joint_commands)->data.size() != command_interfaces_.size())
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *(get_node()->get_clock()), 1000,
      "command size (%zu) does not match number of interfaces (%zu)",
      (*joint_commands)->data.size(), command_interfaces_.size());
    return controller_interface::return_type::ERROR;
  }

  for (auto index = 0ul; index < command_interfaces_.size(); ++index)
  {
    command_interfaces_[index].set_value((*joint_commands)->data[index]);
  }

  return controller_interface::return_type::OK;
}

void IKController::declare_parameters()
{
  param_listener_ = std::make_shared<ParamListener>(get_node());
}

controller_interface::CallbackReturn IKController::read_parameters()
{
  if (!param_listener_)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during init");
    return controller_interface::CallbackReturn::ERROR;
  }
  params_ = param_listener_->get_params();

  if (params_.joints.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter was empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (params_.interface_name.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'interface_name' parameter was empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  for (const auto & joint : params_.joints)
  {
    // command_interface_types_.push_back(joint + "/" + params_.interface_name); // This comes from the YAML parameter. Does it auto switch the word 'position' to HW_IF_POSITION?
    command_interface_types_.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    state_interface_types_.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

}  // namespace base_package

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  base_package::IKController, controller_interface::ControllerInterface)
