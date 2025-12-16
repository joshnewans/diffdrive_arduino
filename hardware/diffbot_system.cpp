// Copyright 2021 ros2_control Development Team
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

#include "diffdrive_arduino/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace diffdrive_arduino
{
hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // BEGIN: Custom code
  cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
  cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);
  if (info_.hardware_parameters.count("pid_p") > 0)
  {
    cfg_.pid_p = std::stoi(info_.hardware_parameters["pid_p"]);
    cfg_.pid_d = std::stoi(info_.hardware_parameters["pid_d"]);
    cfg_.pid_i = std::stoi(info_.hardware_parameters["pid_i"]);
    cfg_.pid_o = std::stoi(info_.hardware_parameters["pid_o"]);
  }
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "PID values not supplied, using defaults.");
  }
  

  wheel_l_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
  wheel_r_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);
  // END: Custom code

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        get_logger(),
        "Joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have '%s' as first state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have '%s' as second state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: Custom code
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Configuring ...please wait...");
  if (comms_.connected())
  {
    comms_.disconnect();
  }
  comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully configured!");
  //END: Custom code

  // reset values always when configuring hardware
  for (const auto & [name, descr] : joint_state_interfaces_)
  {
    set_state(name, 0.0);
  }
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    set_command(name, 0.0);
  }
  RCLCPP_INFO(get_logger(), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: Custom code
  RCLCPP_INFO(get_logger(), "Activating ...please wait...");
  if (!comms_.connected())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (cfg_.pid_p > 0)
  {
    comms_.set_pid_values(cfg_.pid_p,cfg_.pid_d,cfg_.pid_i,cfg_.pid_o);
    RCLCPP_INFO(get_logger(), "PID values set: P=%f, D=%f, I=%f, O=%f", cfg_.pid_p, cfg_.pid_d, cfg_.pid_i, cfg_.pid_o);
  }
  RCLCPP_INFO(get_logger(), "Successfully activated!");
  //END: Custom code

  // command and state should be equal when starting
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    set_command(name, get_state(name));
  }

  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");
  RCLCPP_INFO(get_logger(), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffDriveArduinoHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // Begin: Custom code
  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  comms_.read_encoder_values(wheel_l_.enc, wheel_r_.enc);

  double delta_seconds = period.seconds();

  double pos_prev = wheel_l_.pos;
  wheel_l_.pos = wheel_l_.calc_enc_angle();
  wheel_l_.vel = (wheel_l_.pos - pos_prev) / delta_seconds;

  pos_prev = wheel_r_.pos;
  wheel_r_.pos = wheel_r_.calc_enc_angle();
  wheel_r_.vel = (wheel_r_.pos - pos_prev) / delta_seconds;
  
  set_state(wheel_l_.name + "/" + hardware_interface::HW_IF_VELOCITY, wheel_l_.vel);
  set_state(wheel_r_.name + "/" + hardware_interface::HW_IF_VELOCITY, wheel_r_.vel);

  set_state(wheel_l_.name + "/" + hardware_interface::HW_IF_POSITION, wheel_l_.pos);
  set_state(wheel_r_.name + "/" + hardware_interface::HW_IF_POSITION, wheel_r_.pos);
  // END: Custom code

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type diffdrive_arduino ::DiffDriveArduinoHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

  // BEGIN: Custom code
  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }
  wheel_l_.cmd = get_command(wheel_l_.name + "/" + hardware_interface::HW_IF_VELOCITY);
  wheel_r_.cmd = get_command(wheel_r_.name + "/" + hardware_interface::HW_IF_VELOCITY);
  
  // RCLCPP_INFO_THROTTLE(
  //   get_logger(), *get_clock(), 500,
  //   "Motor cmd: left = %f, right = %f",
  //   wheel_l_.cmd, wheel_r_.cmd);
  
  int motor_l_counts_per_loop = static_cast<int>(wheel_l_.cmd / wheel_l_.rads_per_count / cfg_.loop_rate);
  int motor_r_counts_per_loop = static_cast<int>(wheel_r_.cmd / wheel_r_.rads_per_count / cfg_.loop_rate);
  // RCLCPP_INFO_THROTTLE(
  //   get_logger(), *get_clock(), 500,
  //   "Motor counts per loop: left = %d, right = %d",
  //   motor_l_counts_per_loop, motor_r_counts_per_loop);
  comms_.set_motor_values(motor_l_counts_per_loop, motor_r_counts_per_loop);
  return hardware_interface::return_type::OK;
  // END: Custom code
  return hardware_interface::return_type::OK;
}

}  // namespace diffdrive_arduino

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  diffdrive_arduino::DiffDriveArduinoHardware, hardware_interface::SystemInterface)
