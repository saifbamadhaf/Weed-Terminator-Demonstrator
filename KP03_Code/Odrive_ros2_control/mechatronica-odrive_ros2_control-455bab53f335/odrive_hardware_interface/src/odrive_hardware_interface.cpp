// Copyright 2021 Factor Robotics
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

#include "odrive_hardware_interface/odrive_hardware_interface.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace odrive_hardware_interface
{
return_type ODriveHardwareInterface::configure(const hardware_interface::HardwareInfo& info)
{
  if (configure_default(info) != return_type::OK)
  {
    return return_type::ERROR;
  }

  std::unordered_map<std::string, double> reduction_map;
  std::unordered_map<std::string, double> joint_offset_map;
  std::unordered_map<std::string, std::string> role_map;
  std::vector<std::string> joint_names;
  
  serial_numbers_.resize(2);

  actuator_offset_.resize(info_.joints.size(), 0.0);
  joint_offset_.resize(info_.joints.size(), 0.0);
  reduction_.resize(info_.joints.size(), 1.0);

  hw_vbus_voltages_.resize(info_.sensors.size(), std::numeric_limits<double>::quiet_NaN());

  joint_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  joint_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  joint_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  joint_commands_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  hw_axis_errors_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_motor_errors_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_encoder_errors_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_controller_errors_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_fet_temperatures_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_motor_temperatures_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::TransmissionInfo& transmission : info_.transmissions)
  {
    transmission_in_use_ = transmission_type_t::SIMPLE;
    if (transmission.type == "TBotTransmission")
    {
      transmission_in_use_ = transmission_type_t::TBOT;
    }
    if (transmission.type == "AddTransmission")
    {
      transmission_in_use_ = transmission_type_t::ADD;
    }

    for (const hardware_interface::JointInfo& joint : transmission.joints)
    {
      joint_offset_map[joint.name] = joint.offset;
      reduction_map[joint.name] = joint.mechanical_reduction;
      role_map[joint.name] = joint.role;
    }
    for (const hardware_interface::ActuatorInfo& actuator : transmission.actuators)
    {
      actuator_offset_.emplace_back(actuator.offset);
      role_map[actuator.name] = actuator.role;
    }
  }
  
  std::cout << " TRANSMISSION--";
  switch (transmission_in_use_)
  {
    case transmission_type_t::SIMPLE:
      std::cout << "Simple transmission";
      break;
    case transmission_type_t::TBOT:
      std::cout << "TBot transmission";
      break;
    case transmission_type_t::ADD:
      std::cout << "Add transmission";
      break;
    default:
      std::cout << "<no transmission>";
      break;

  }
  std::cout << std::endl;

  for (const hardware_interface::ComponentInfo& sensor : info_.sensors)
  {
    serial_numbers_[0].emplace_back(std::stoull(sensor.parameters.at("serial_number"), 0, 16));
  }

  for (const hardware_interface::ComponentInfo& joint : info_.joints)
  {
    serial_numbers_[1].emplace_back(std::stoull(joint.parameters.at("serial_number"), 0, 16));
    axes_.emplace_back(std::stoi(joint.parameters.at("axis")));
    enable_watchdogs_.emplace_back(std::stoi(joint.parameters.at("enable_watchdog")));
    joint_names.emplace_back(joint.name);
  }

  odrive = new ODriveUSB();
  CHECK(odrive->init(serial_numbers_));

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    std::cout << "Joint " << i << ": " << joint_names.at(i) << " (axis " << axes_[i] << ")" << std::endl;
    float torque_constant;
    CHECK(odrive->read(serial_numbers_[1][i], AXIS__MOTOR__CONFIG__TORQUE_CONSTANT + per_axis_offset * axes_[i],
                       torque_constant));
    torque_constants_.emplace_back(torque_constant);

    if (reduction_map.count(joint_names[i]) > 0)
    {
      reduction_[i] = reduction_map.at(joint_names[i]);
      joint_offset_[i] = joint_offset_map.at(joint_names[i]);
    }

    if (enable_watchdogs_[i])
    {
      CHECK(odrive->write(serial_numbers_[1][i], AXIS__CONFIG__WATCHDOG_TIMEOUT + per_axis_offset * axes_[i],
                          std::stof(info_.joints[i].parameters.at("watchdog_timeout"))));
    }
    CHECK(odrive->write(serial_numbers_[1][i], AXIS__CONFIG__ENABLE_WATCHDOG + per_axis_offset * axes_[i],
                        (bool)enable_watchdogs_[i]));
  }

  control_level_.resize(info_.joints.size(), integration_level_t::UNDEFINED);
  status_ = hardware_interface::status::CONFIGURED;
  return return_type::OK;
}

std::vector<hardware_interface::StateInterface> ODriveHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.sensors.size(); i++)
  {
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(info_.sensors[i].name, "vbus_voltage", &hw_vbus_voltages_[i]));
  }

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_positions_[i]));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(info_.joints[i].name, "axis_error", &hw_axis_errors_[i]));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(info_.joints[i].name, "motor_error", &hw_motor_errors_[i]));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(info_.joints[i].name, "encoder_error", &hw_encoder_errors_[i]));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(info_.joints[i].name, "controller_error", &hw_controller_errors_[i]));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(info_.joints[i].name, "fet_temperature", &hw_fet_temperatures_[i]));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(info_.joints[i].name, "motor_temperature", &hw_motor_temperatures_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ODriveHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_efforts_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_commands_velocities_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_commands_positions_[i]));
  }

  return command_interfaces;
}

return_type ODriveHardwareInterface::prepare_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                                 const std::vector<std::string>& stop_interfaces)
{
  std::vector<integration_level_t> new_modes = control_level_;
  for (std::string key : stop_interfaces)
  {
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
      if (key.find(info_.joints[i].name) != std::string::npos)
      {
        new_modes[i] = integration_level_t::UNDEFINED;
      }
    }
  }

  for (std::string key : start_interfaces)
  {
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION)
      {
        new_modes[i] = integration_level_t::POSITION;
        //std::cout << "TRANSMISSION DEBUG: " << info_.joints[i].name << " POSITION" << std::endl;
      }
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY)
      {
        new_modes[i] = integration_level_t::VELOCITY;
        //std::cout << "TRANSMISSION DEBUG: " << info_.joints[i].name << " VELOCITY" << std::endl;
      }
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT)
      {
        new_modes[i] = integration_level_t::EFFORT;
        //std::cout << "TRANSMISSION DEBUG: " << info_.joints[i].name << " EFFORT" << std::endl;
      }
    }
  }

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    if (control_level_[i] != new_modes[i])
    {
      float input_torque, input_vel, input_pos;
      int32_t requested_state, control_mode;

      switch (new_modes[i])
      {
        case integration_level_t::UNDEFINED:
          requested_state = AXIS_STATE_IDLE;
          CHECK(odrive->write(serial_numbers_[1][i], AXIS__REQUESTED_STATE + per_axis_offset * axes_[i],
                              requested_state));
          break;

        case integration_level_t::EFFORT:
          hw_commands_efforts_[i] = hw_efforts_[i];
          control_mode = (int32_t)new_modes[i];
          CHECK(odrive->write(serial_numbers_[1][i],
                              AXIS__CONTROLLER__CONFIG__CONTROL_MODE + per_axis_offset * axes_[i], control_mode));
          input_torque = hw_commands_efforts_[i];
          CHECK(odrive->write(serial_numbers_[1][i], AXIS__CONTROLLER__INPUT_TORQUE + per_axis_offset * axes_[i],
                              input_torque));
          requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
          CHECK(odrive->write(serial_numbers_[1][i], AXIS__REQUESTED_STATE + per_axis_offset * axes_[i],
                              requested_state));
          break;

        case integration_level_t::VELOCITY:
          hw_commands_velocities_[i] = hw_velocities_[i];
          joint_commands_velocities_[i] = joint_velocities_[i];
          hw_commands_efforts_[i] = 0;
          control_mode = (int32_t)new_modes[i];
          CHECK(odrive->write(serial_numbers_[1][i],
                              AXIS__CONTROLLER__CONFIG__CONTROL_MODE + per_axis_offset * axes_[i], control_mode));
          input_vel = hw_commands_velocities_[i] / 2 / M_PI;
          CHECK(odrive->write(serial_numbers_[1][i], AXIS__CONTROLLER__INPUT_VEL + per_axis_offset * axes_[i],
                              input_vel));
          input_torque = hw_commands_efforts_[i];
          CHECK(odrive->write(serial_numbers_[1][i], AXIS__CONTROLLER__INPUT_TORQUE + per_axis_offset * axes_[i],
                              input_torque));
          requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
          CHECK(odrive->write(serial_numbers_[1][i], AXIS__REQUESTED_STATE + per_axis_offset * axes_[i],
                              requested_state));
          break;

        case integration_level_t::POSITION:
          hw_commands_positions_[i] = hw_positions_[i];
          joint_commands_positions_[i] = joint_positions_[i];
          hw_commands_velocities_[i] = 0;
          joint_commands_velocities_[i] = 0;
          hw_commands_efforts_[i] = 0;
          control_mode = (int32_t)new_modes[i];
          CHECK(odrive->write(serial_numbers_[1][i],
                              AXIS__CONTROLLER__CONFIG__CONTROL_MODE + per_axis_offset * axes_[i], control_mode));
          input_pos = hw_commands_positions_[i] / 2 / M_PI;
          CHECK(odrive->write(serial_numbers_[1][i], AXIS__CONTROLLER__INPUT_POS + per_axis_offset * axes_[i],
                              input_pos));
          input_vel = hw_commands_velocities_[i] / 2 / M_PI;
          CHECK(odrive->write(serial_numbers_[1][i], AXIS__CONTROLLER__INPUT_VEL + per_axis_offset * axes_[i],
                              input_vel));
          input_torque = hw_commands_efforts_[i];
          CHECK(odrive->write(serial_numbers_[1][i], AXIS__CONTROLLER__INPUT_TORQUE + per_axis_offset * axes_[i],
                              input_torque));
          requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
          CHECK(odrive->write(serial_numbers_[1][i], AXIS__REQUESTED_STATE + per_axis_offset * axes_[i],
                              requested_state));
          break;
      }
    }
    control_level_[i] = new_modes[i];
  }

  return return_type::OK;
}

return_type ODriveHardwareInterface::start()
{
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    if (enable_watchdogs_[i])
    {
      CHECK(odrive->call(serial_numbers_[1][i], AXIS__WATCHDOG_FEED + per_axis_offset * axes_[i]));
    }
    CHECK(odrive->call(serial_numbers_[1][i], CLEAR_ERRORS));
  }

  status_ = hardware_interface::status::STARTED;
  return return_type::OK;
}

return_type ODriveHardwareInterface::stop()
{
  int32_t requested_state = AXIS_STATE_IDLE;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    CHECK(odrive->write(serial_numbers_[1][i], AXIS__REQUESTED_STATE + per_axis_offset * axes_[i], requested_state));
  }

  status_ = hardware_interface::status::STOPPED;
  return return_type::OK;
}

return_type ODriveHardwareInterface::read()
{
  for (size_t i = 0; i < info_.sensors.size(); i++)
  {
    float vbus_voltage;

    CHECK(odrive->read(serial_numbers_[0][i], VBUS_VOLTAGE, vbus_voltage));
    hw_vbus_voltages_[i] = vbus_voltage;
  }

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    float Iq_measured, vel_estimate, pos_estimate, fet_temperature, motor_temperature;
    uint8_t controller_error;
    uint16_t encoder_error;
    uint32_t axis_error;
    uint64_t motor_error;

    CHECK(odrive->read(serial_numbers_[1][i], AXIS__MOTOR__CURRENT_CONTROL__IQ_MEASURED + per_axis_offset * axes_[i],
                       Iq_measured));
    hw_efforts_[i] = Iq_measured * torque_constants_[i];

    CHECK(odrive->read(serial_numbers_[1][i], AXIS__ENCODER__VEL_ESTIMATE + per_axis_offset * axes_[i], vel_estimate));
    hw_velocities_[i] = vel_estimate * 2 * M_PI;

    CHECK(odrive->read(serial_numbers_[1][i], AXIS__ENCODER__POS_ESTIMATE + per_axis_offset * axes_[i], pos_estimate));
    hw_positions_[i] = pos_estimate * 2 * M_PI;

    CHECK(odrive->read(serial_numbers_[1][i], AXIS__ERROR + per_axis_offset * axes_[i], axis_error));
    hw_axis_errors_[i] = axis_error;

    CHECK(odrive->read(serial_numbers_[1][i], AXIS__MOTOR__ERROR + per_axis_offset * axes_[i], motor_error));
    hw_motor_errors_[i] = motor_error;

    CHECK(odrive->read(serial_numbers_[1][i], AXIS__ENCODER__ERROR + per_axis_offset * axes_[i], encoder_error));
    hw_encoder_errors_[i] = encoder_error;

    CHECK(odrive->read(serial_numbers_[1][i], AXIS__CONTROLLER__ERROR + per_axis_offset * axes_[i], controller_error));
    hw_controller_errors_[i] = controller_error;

    CHECK(odrive->read(serial_numbers_[1][i], AXIS__MOTOR__FET_THERMISTOR__TEMPERATURE + per_axis_offset * axes_[i],
                       fet_temperature));
    hw_fet_temperatures_[i] = fet_temperature;

    CHECK(odrive->read(serial_numbers_[1][i], AXIS__MOTOR__MOTOR_THERMISTOR__TEMPERATURE + per_axis_offset * axes_[i],
                       motor_temperature));
    hw_motor_temperatures_[i] = motor_temperature;
  }

  actuator_to_joint(hw_positions_, joint_positions_, joint_commands_positions_);
  actuator_to_joint(hw_velocities_, joint_velocities_, joint_commands_positions_);

  return return_type::OK;
}

return_type ODriveHardwareInterface::write()
{
  joint_to_actuator(joint_commands_positions_, hw_commands_positions_);
  joint_to_actuator(joint_commands_velocities_, hw_commands_velocities_);

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    float input_torque, input_vel, input_pos;

    switch (control_level_[i])
    {
      case integration_level_t::POSITION:
        input_pos = hw_commands_positions_[i] / 2 / M_PI;
        CHECK(
            odrive->write(serial_numbers_[1][i], AXIS__CONTROLLER__INPUT_POS + per_axis_offset * axes_[i], input_pos));

      case integration_level_t::VELOCITY:
        input_vel = hw_commands_velocities_[i] / 2 / M_PI;
        CHECK(
            odrive->write(serial_numbers_[1][i], AXIS__CONTROLLER__INPUT_VEL + per_axis_offset * axes_[i], input_vel));

      case integration_level_t::EFFORT:
        input_torque = hw_commands_efforts_[i];
        CHECK(odrive->write(serial_numbers_[1][i], AXIS__CONTROLLER__INPUT_TORQUE + per_axis_offset * axes_[i],
                            input_torque));

      case integration_level_t::UNDEFINED:
        if (enable_watchdogs_[i])
        {
          CHECK(odrive->call(serial_numbers_[1][i], AXIS__WATCHDOG_FEED + per_axis_offset * axes_[i]));
        }
    }
  }

  return return_type::OK;
}

void ODriveHardwareInterface::actuator_to_joint(std::vector<double> actuator_values, std::vector<double>& joint_values, std::vector<double> joint_commands)
{
  switch (transmission_in_use_)
  {
    case transmission_type_t::SIMPLE:
      for (size_t i = 0; i < actuator_values.size(); i++)
      {
        joint_values[i] = actuator_values[i] / reduction_[i];
      }
      break;
    case transmission_type_t::TBOT:
      joint_values[0] = 0.5 * (actuator_values[0] / reduction_[0] + actuator_values[1] / reduction_[1]);
      joint_values[1] = 0.5 * (actuator_values[0] / reduction_[0] - actuator_values[1] / reduction_[1]);
      break;
    case transmission_type_t::ADD:

        if (std::isnan(joint_commands[0])){
          joint_values[0] = actuator_values[0] / reduction_[0];
        }
        else{
          //joint_values[0] = joint_commands[0];
          //provide proper feedback for plan joint by subtracting compensation command
          joint_values[0] = actuator_values[0] / reduction_[0] - joint_commands[1];
        }

        if (std::isnan(joint_commands[1])){
          joint_values[1] = 0;
        }
        else{
          joint_values[1] = joint_commands[1];
        }

        joint_values[2] = actuator_values[2] / reduction_[2];

      break;
    default:
      for (size_t i = 0; i < actuator_values.size(); i++)
      {
        joint_values[i] = actuator_values[i];
      }
  }
}

void ODriveHardwareInterface::joint_to_actuator(std::vector<double> joint_values, std::vector<double>& actuator_values)
{
  switch (transmission_in_use_)
  {
    case transmission_type_t::SIMPLE:
      for (size_t i = 0; i < joint_values.size(); i++)
      {
        actuator_values[i] = joint_values[i] * reduction_[i];
      }
      break;
    case transmission_type_t::TBOT:
      actuator_values[0] = (joint_values[0] + joint_values[1]) * reduction_[0];
      actuator_values[1] = (joint_values[0] - joint_values[1]) * reduction_[1];
      break;
    case transmission_type_t::ADD:
        actuator_values[0] = joint_values[0] * reduction_[0] + joint_values[1] * reduction_[1];
        actuator_values[1] = joint_values[0] * reduction_[0] + joint_values[1] * reduction_[1];
        actuator_values[2] = joint_values[2] * reduction_[2];

        // Limit x axis to within travel for safety
        if (actuator_values[0] > 0){
          std::cout << "TRANSMISSION DEBUG:" << "x axis movement limited, requested (invalid) position: " << actuator_values[0] << std::endl;
          actuator_values[0] = 0;
          actuator_values[1] = 0;
        }

        if (actuator_values[0] < -120){
          std::cout << "TRANSMISSION DEBUG:" << "x axis movement limited, requested (invalid) position: " << actuator_values[0] << std::endl;
          actuator_values[0] = -120;
          actuator_values[1] = -120;
        }


        // std::cout << "TRANSMISSION DEBUG:" << "joint size: " << joint_values.size() << std::endl;
        // std::cout << "TRANSMISSION DEBUG:" << "joint_values[0]: "<< joint_values[0] << std::endl;
        // std::cout << "TRANSMISSION DEBUG:" << "joint_values[1]: "<< joint_values[1] << std::endl;
        // std::cout << "TRANSMISSION DEBUG:" << "joint_values[2]: "<< joint_values[2] << std::endl;
        // std::cout << "TRANSMISSION DEBUG:" << "actuator_values[0]: "<< actuator_values[0] << std::endl;
        // std::cout << "TRANSMISSION DEBUG:" << "actuator_values[1]: "<< actuator_values[1] << std::endl;
        
      break;
    default:
      for (size_t i = 0; i < joint_values.size(); i++)
      {
        actuator_values[i] = joint_values[i];
      }
  }
}

}  // namespace odrive_hardware_interface

PLUGINLIB_EXPORT_CLASS(odrive_hardware_interface::ODriveHardwareInterface, hardware_interface::SystemInterface)
