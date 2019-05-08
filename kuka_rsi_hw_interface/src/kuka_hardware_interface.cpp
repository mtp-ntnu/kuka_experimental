/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014 Norwegian University of Science and Technology
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Norwegian University of Science and
 *     Technology, nor the names of its contributors may be used to
 *     endorse or promote products derived from this software without
 *     specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 * Author: Lars Tingelstad <lars.tingelstad@ntnu.no>
 */

#include <kuka_rsi_hw_interface/kuka_hardware_interface.h>

#include <stdexcept>

namespace kuka_rsi_hw_interface
{
KukaHardwareInterface::KukaHardwareInterface()
  : joint_position_(6, 0.0)
  , joint_velocity_(6, 0.0)
  , joint_effort_(6, 0.0)
  , joint_position_command_(6, 0.0)
  , joint_velocity_command_(6, 0.0)
  , joint_effort_command_(6, 0.0)
  , joint_names_(6)
  , last_joint_position_(6, 0.0)
  , rsi_initial_joint_positions_(6, 0.0)
  , rsi_joint_position_corrections_(6, 0.0)
  , ipoc_(0)
  , n_dof_(6)
{
  in_buffer_.resize(1024);
  out_buffer_.resize(1024);
  remote_host_.resize(1024);
  remote_port_.resize(1024);

  if (!nh_.getParam("controller_joint_names", joint_names_))
  {
    ROS_ERROR("Cannot find required parameter 'controller_joint_names' "
              "on the parameter server.");
    throw std::runtime_error("Cannot find required parameter "
                             "'controller_joint_names' on the parameter server.");
  }

  // RML
  rml_.reset(new ReflexxesAPI(n_dof_, 0.004));
  rml_input_.reset(new RMLPositionInputParameters(n_dof_));
  rml_output_.reset(new RMLPositionOutputParameters(n_dof_));

  // Create ros_control interfaces
  for (std::size_t i = 0; i < n_dof_; ++i)
  {
    // Create joint state interface for all joints
    joint_state_interface_.registerHandle(hardware_interface::JointStateHandle(joint_names_[i], &joint_position_[i],
                                                                               &joint_velocity_[i], &joint_effort_[i]));

    // Create joint position control interface
    posvel_joint_interface_.registerHandle(hardware_interface::PosVelJointHandle(
        joint_state_interface_.getHandle(joint_names_[i]), &joint_position_command_[i], &joint_velocity_command_[i]));

    joint_limits_interface::JointLimits joint_limits;
    const bool rosparam_limits_ok = joint_limits_interface::getJointLimits(joint_names_[i], nh_, joint_limits);
    if (!rosparam_limits_ok)
    {
      ROS_ERROR("Cannot find required parameter 'joint_limits' on the parameter server.");
      throw std::runtime_error("Cannot find required parameter 'joint_limits' on the parameter server.");
    }
    // position_joint_limit_saturation_interface_.registerHandle(joint_limits_interface::PositionJointSaturationHandle(
    //     position_joint_interface_.getHandle(joint_names_[i]), joint_limits));

    if (joint_limits.has_velocity_limits)
    {
      rml_input_->MaxVelocityVector->VecData[i] = joint_limits.max_velocity;
    }
    if (joint_limits.has_acceleration_limits)
    {
      rml_input_->MaxAccelerationVector->VecData[i] = joint_limits.max_acceleration;
    }
  }

  // Register interfaces
  registerInterface(&joint_state_interface_);
  registerInterface(&posvel_joint_interface_);

  ROS_INFO_STREAM_NAMED("hardware_interface", "Loaded kuka_rsi_hardware_interface");
}

KukaHardwareInterface::~KukaHardwareInterface()
{
}

bool KukaHardwareInterface::read(const ros::Time time, const ros::Duration period)
{
  in_buffer_.resize(1024);

  if (server_->recv(in_buffer_) == 0)
  {
    return false;
  }

  if (rt_rsi_pub_->trylock())
  {
    rt_rsi_pub_->msg_.data = in_buffer_;
    rt_rsi_pub_->unlockAndPublish();
  }

  rsi_state_ = RSIState(in_buffer_);
  for (std::size_t i = 0; i < n_dof_; ++i)
  {
    joint_position_[i] = DEG2RAD * rsi_state_.positions[i];
    if (!period.isZero())
    {
      // joint_velocity_[i] = (joint_position_[i] - last_joint_position_[i]) / period.toSec();
    }
  }
  ipoc_ = rsi_state_.ipoc;
  last_joint_position_ = joint_position_;

  return true;
}

bool KukaHardwareInterface::write(const ros::Time time, const ros::Duration period)
{
  out_buffer_.resize(1024);

  // position_joint_limit_saturation_interface_.enforceLimits(period);

  for (std::size_t i = 0; i < n_dof_; ++i)
  {
    rml_input_->CurrentPositionVector->VecData[i] = joint_position_[i];
    rml_input_->CurrentVelocityVector->VecData[i] = joint_velocity_[i];

    rml_input_->TargetPositionVector->VecData[i] = joint_position_command_[i];
    rml_input_->TargetVelocityVector->VecData[i] = joint_velocity_command_[i];
  }
  int result = rml_->RMLPosition(*rml_input_.get(), rml_output_.get(), rml_flags_);
  if (result < 0)
  {
    ROS_ERROR_STREAM("rml error: " << result);
  }

  for (std::size_t i = 0; i < n_dof_; ++i)
  {
    joint_position_command_[i] = rml_output_->NewPositionVector->VecData[i];
    joint_velocity_command_[i] = rml_output_->NewVelocityVector->VecData[i];

    rsi_joint_position_corrections_[i] = (RAD2DEG * joint_position_command_[i]) - rsi_initial_joint_positions_[i];

    joint_velocity_[i] = joint_velocity_command_[i];
    joint_effort_[i] = rml_output_->NewAccelerationVector->VecData[i];
  }

  out_buffer_ = RSICommand(rsi_joint_position_corrections_, ipoc_).xml_doc;
  server_->send(out_buffer_);

  return true;
}

void KukaHardwareInterface::start()
{
  // Wait for connection from robot
  server_.reset(new UDPServer(local_host_, local_port_));

  ROS_INFO_STREAM_NAMED("kuka_hardware_interface", "Waiting for robot!");

  int bytes = server_->recv(in_buffer_);

  // Drop empty <rob> frame with RSI <= 2.3
  if (bytes < 100)
  {
    bytes = server_->recv(in_buffer_);
  }

  rsi_state_ = RSIState(in_buffer_);
  for (std::size_t i = 0; i < n_dof_; ++i)
  {
    joint_position_[i] = DEG2RAD * rsi_state_.positions[i];
    joint_position_command_[i] = joint_position_[i];
    last_joint_position_[i] = joint_position_[i];
    rsi_initial_joint_positions_[i] = rsi_state_.initial_positions[i];

    rml_input_->CurrentPositionVector->VecData[i] = joint_position_[i];
    rml_input_->CurrentVelocityVector->VecData[i] = 0.0;
    rml_input_->CurrentAccelerationVector->VecData[i] = 0.0;
    rml_input_->SelectionVector->VecData[i] = true;
  }

  ipoc_ = rsi_state_.ipoc;
  out_buffer_ = RSICommand(rsi_joint_position_corrections_, ipoc_).xml_doc;
  server_->send(out_buffer_);
  // Set receive timeout to 1 second
  server_->set_timeout(1000);
  ROS_INFO_STREAM_NAMED("kuka_hardware_interface", "Got connection from robot");
}

void KukaHardwareInterface::configure()
{
  const std::string param_addr = "rsi/listen_address";
  const std::string param_port = "rsi/listen_port";

  if (nh_.getParam(param_addr, local_host_) && nh_.getParam(param_port, local_port_))
  {
    ROS_INFO_STREAM_NAMED("kuka_hardware_interface",
                          "Setting up RSI server on: (" << local_host_ << ", " << local_port_ << ")");
  }
  else
  {
    std::string msg = "Failed to get RSI listen address or listen port from"
                      " parameter server (looking for '" +
                      param_addr + "' and '" + param_port + "')";
    ROS_ERROR_STREAM(msg);
    throw std::runtime_error(msg);
  }
  rt_rsi_pub_.reset(new realtime_tools::RealtimePublisher<std_msgs::String>(nh_, "rsi_xml_doc", 3));
}

}  // namespace kuka_rsi_hw_interface
