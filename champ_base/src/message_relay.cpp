/*
Copyright (c) 2019-2020, Juan Miguel Jimeno
Copyright (c) 2021, Fetullah Atas
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <message_relay.hpp>

MessageRelay::MessageRelay()
: Node("message_relay_node")
{
  imu_data_.orientation.w = 1.0;

  auto custom_qos = rclcpp::QoS(10);

  foot_contacts_publisher_ = this->create_publisher<champ_msgs::msg::ContactsStamped>(
    "foot_contacts", custom_qos);
  joint_states_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "joint_states",
    custom_qos);
  joint_commands_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "joint_trajectory_controller/joint_trajectory", custom_qos);

  imu_raw_subscriber_ = this->create_subscription<champ_msgs::msg::Imu>(
    "imu/raw",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&MessageRelay::IMURawCallback, this, std::placeholders::_1));

  joints_raw_subscriber_ = this->create_subscription<champ_msgs::msg::Joints>(
    "joint_states/raw",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&MessageRelay::jointStatesRawCallback, this, std::placeholders::_1));

  foot_contacts_subscriber_ = this->create_subscription<champ_msgs::msg::Contacts>(
    "foot_contacts/raw",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&MessageRelay::footContactCallback, this, std::placeholders::_1));

  declare_parameter("gazebo", true);
  declare_parameter("has_imu", false);
  rclcpp::Parameter use_sim_time("use_sim_time", rclcpp::ParameterValue(true) );
  set_parameter(use_sim_time);

  get_parameter("gazebo", in_gazebo_);
  get_parameter("has_imu", has_imu_);

  //joint_names_ = champ::URDF::getJointNames(nh);
  joint_names_ = {
    "lf_hip_joint",
    "lf_upper_leg_joint",
    "lf_lower_leg_joint",
    "lh_hip_joint",
    "lh_upper_leg_joint",
    "lh_lower_leg_joint",
    "rf_hip_joint",
    "rf_upper_leg_joint",
    "rf_lower_leg_joint",
    "rh_hip_joint",
    "rh_upper_leg_joint",
    "rh_lower_leg_joint"};

  if (has_imu_) {
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", custom_qos);
    mag_publisher_ = this->create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", custom_qos);
  }

  imu_frame_ = node_namespace_ + "imu_link";
}

void MessageRelay::IMURawCallback(const champ_msgs::msg::Imu::ConstSharedPtr msg)
{
  sensor_msgs::msg::Imu imu_data_msg;
  sensor_msgs::msg::MagneticField imu_mag_msg;

  imu_data_msg.header.stamp = this->now();
  imu_data_msg.header.frame_id = imu_frame_;

  imu_data_msg.orientation.w = msg->orientation.w;
  imu_data_msg.orientation.x = msg->orientation.x;
  imu_data_msg.orientation.y = msg->orientation.y;
  imu_data_msg.orientation.z = msg->orientation.z;

  imu_data_msg.linear_acceleration.x = msg->linear_acceleration.x;
  imu_data_msg.linear_acceleration.y = msg->linear_acceleration.y;
  imu_data_msg.linear_acceleration.z = msg->linear_acceleration.z;

  imu_data_msg.angular_velocity.x = msg->angular_velocity.x;
  imu_data_msg.angular_velocity.y = msg->angular_velocity.y;
  imu_data_msg.angular_velocity.z = msg->angular_velocity.z;

  imu_data_msg.orientation_covariance[0] = 0.0025;
  imu_data_msg.orientation_covariance[4] = 0.0025;
  imu_data_msg.orientation_covariance[8] = 0.0025;

  imu_data_msg.angular_velocity_covariance[0] = 0.000001;
  imu_data_msg.angular_velocity_covariance[4] = 0.000001;
  imu_data_msg.angular_velocity_covariance[8] = 0.000001;

  imu_data_msg.linear_acceleration_covariance[0] = 0.0001;
  imu_data_msg.linear_acceleration_covariance[4] = 0.0001;
  imu_data_msg.linear_acceleration_covariance[8] = 0.0001;

  imu_mag_msg.header.stamp = this->now();
  imu_mag_msg.header.frame_id = imu_frame_;

  imu_mag_msg.magnetic_field.x = msg->magnetic_field.x;
  imu_mag_msg.magnetic_field.y = msg->magnetic_field.y;
  imu_mag_msg.magnetic_field.z = msg->magnetic_field.z;

  imu_mag_msg.magnetic_field_covariance[0] = 0.000001;
  imu_mag_msg.magnetic_field_covariance[4] = 0.000001;
  imu_mag_msg.magnetic_field_covariance[8] = 0.000001;

  if (has_imu_) {
    imu_publisher_->publish(imu_data_msg);
    mag_publisher_->publish(imu_mag_msg);
  }
}

void MessageRelay::jointStatesRawCallback(const champ_msgs::msg::Joints::ConstSharedPtr msg)
{
  if (in_gazebo_) {
    trajectory_msgs::msg::JointTrajectory joints_cmd_msg;
    joints_cmd_msg.header.stamp = this->now();
    joints_cmd_msg.joint_names = joint_names_;

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.resize(12);

    point.time_from_start = rclcpp::Duration(1.0 / 60.0);
    for (size_t i = 0; i < 12; i++) {
      point.positions[i] = msg->position[i];
    }

    joints_cmd_msg.points.push_back(point);
    joint_commands_publisher_->publish(joints_cmd_msg);
  } else {
    sensor_msgs::msg::JointState joints_msg;

    joints_msg.header.stamp = this->now();
    joints_msg.name.resize(joint_names_.size());
    joints_msg.position.resize(joint_names_.size());
    joints_msg.name = joint_names_;

    for (size_t i = 0; i < joint_names_.size(); ++i) {
      joints_msg.position[i] = msg->position[i];
    }

    joint_states_publisher_->publish(joints_msg);
  }
}

void MessageRelay::footContactCallback(const champ_msgs::msg::Contacts::ConstSharedPtr msg)
{
  champ_msgs::msg::ContactsStamped contacts_msg;
  contacts_msg.header.stamp = this->now();
  contacts_msg.contacts.resize(4);

  for (size_t i = 0; i < 4; i++) {
    contacts_msg.contacts[i] = msg->contacts[i];
  }

  foot_contacts_publisher_->publish(contacts_msg);
}

int main(int argc, char const * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MessageRelay>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
