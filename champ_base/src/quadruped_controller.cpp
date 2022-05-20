/*
Copyright (c) 2019-2020, Juan Miguel Jimeno
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

#include <quadruped_controller.hpp>

champ::PhaseGenerator::Time rosTimeToChampTime(const rclcpp::Time & time)
{
  return time.nanoseconds() / 1000ul;
}


QuadrupedController::QuadrupedController()
: Node("quadruped_controller_node"),
  body_controller_(base_),
  leg_controller_(base_, rosTimeToChampTime(this->now())),
  kinematics_(base_)
{
  std::string joint_control_topic;
  std::string knee_orientation;
  double loop_rate;

  this->declare_parameter("gait.pantograph_leg", false);
  this->declare_parameter("gait.max_linear_velocity_x", 0.5);
  this->declare_parameter("gait.max_linear_velocity_y", 0.25);
  this->declare_parameter("gait.max_angular_velocity_z", 1.0);
  this->declare_parameter("gait.com_x_translation", 0.0);
  this->declare_parameter("gait.swing_height", 0.04);
  this->declare_parameter("gait.stance_depth", 0.00);
  this->declare_parameter("gait.stance_duration", 0.15);
  this->declare_parameter("gait.nominal_height", 0.20);
  this->declare_parameter("gait.knee_orientation", ">>");
  this->declare_parameter("publish_foot_contacts", true);
  this->declare_parameter("publish_joint_states", true);
  this->declare_parameter("publish_joint_control", true);
  this->declare_parameter("gazebo", true);
  this->declare_parameter("joint_controller_topic", "joint_trajectory_controller/joint_trajectory");
  this->declare_parameter("loop_rate", 200.0);

  this->get_parameter("gait.pantograph_leg", gait_config_.pantograph_leg);
  this->get_parameter("gait.max_linear_velocity_x", gait_config_.max_linear_velocity_x);
  this->get_parameter("gait.max_linear_velocity_y", gait_config_.max_linear_velocity_y);
  this->get_parameter("gait.max_angular_velocity_z", gait_config_.max_angular_velocity_z);
  this->get_parameter("gait.com_x_translation", gait_config_.com_x_translation);
  this->get_parameter("gait.swing_height", gait_config_.swing_height);
  this->get_parameter("gait.stance_depth", gait_config_.stance_depth);
  this->get_parameter("gait.stance_duration", gait_config_.stance_duration);
  this->get_parameter("gait.nominal_height", gait_config_.nominal_height);
  this->get_parameter("gait.knee_orientation", knee_orientation);
  this->get_parameter("publish_foot_contacts", publish_foot_contacts_);
  this->get_parameter("publish_joint_states", publish_joint_states_);
  this->get_parameter("publish_joint_control", publish_joint_control_);
  this->get_parameter("gazebo", in_gazebo_);
  this->get_parameter("joint_controller_topic", joint_control_topic);
  this->get_parameter("loop_rate", loop_rate);

  gait_config_.knee_orientation = knee_orientation.c_str();

  base_.setGaitConfig(gait_config_);

  cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&QuadrupedController::cmdVelCallback, this, std::placeholders::_1));

  cmd_pose_subscriber_ =
    this->create_subscription<geometry_msgs::msg::Pose>(
    "body_pose",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&QuadrupedController::cmdPoseCallback, this, std::placeholders::_1));

  if (publish_joint_control_) {
    joint_commands_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      joint_control_topic, 1);
  }
  if (publish_joint_states_ && !in_gazebo_) {
    joint_states_publisher_ =
      this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);
  }
  if (publish_foot_contacts_ && !in_gazebo_) {
    foot_contacts_publisher_ = this->create_publisher<champ_msgs::msg::ContactsStamped>(
      "foot_contacts", 1);
  }

  parameter_client_node_ = std::make_shared<rclcpp::Node>(
    "parameter_client_node");

  rclcpp::SyncParametersClient::SharedPtr parameters_client =
    std::make_shared<rclcpp::SyncParametersClient>(parameter_client_node_, "robot_state_publisher");

  using namespace std::chrono_literals;
  while (!parameters_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        parameter_client_node_->get_logger(),
        "Interrupted while waiting for the parameter service. Exiting.");
      rclcpp::shutdown();
    }
    RCLCPP_INFO(
      parameter_client_node_->get_logger(),
      "Remote parameter service not available, waiting again...");
  }
  urdf::Model model;
  auto parameters = parameters_client->get_parameters(
    {"robot_description"});

  std::string robot_desc;
  for (auto & parameter : parameters) {
    robot_desc = parameter.get_value<std::string>();
  }

  if (!model.initString(robot_desc)) {
    RCLCPP_ERROR(get_logger(), "Failed to parse urdf file");
  }
  RCLCPP_INFO(get_logger(), "Parsed URDF, has got %s name", model.getName().c_str());


  //champ::URDF::loadFromServer(base_, nh);
  std::vector<std::string> links_map;
  links_map.push_back("left_front");
  links_map.push_back("right_front");
  links_map.push_back("left_hind");
  links_map.push_back("right_hind");

  for (int i = 0; i < 4; i++) {
    //fillLeg(base_.legs[i], nh, model, links_map[i]);
    this->declare_parameter(links_map[i]);
    rclcpp::Parameter curr_links_param(links_map[i], std::vector<std::string>({}));
    this->get_parameter(links_map[i], curr_links_param);
    std::vector<std::string> curr_links = curr_links_param.as_string_array();

    for (int j = 3; j > -1; j--) {
      try {
        std::string ref_link;
        std::string end_link;
        if (j > 0) {
          ref_link = curr_links[j - 1];
        } else {
          ref_link = model.getRoot()->name;
        }
        end_link = curr_links[j];

        urdf::Pose pose;
        getPose(&pose, ref_link, end_link, model);
        double x, y, z;
        x = pose.position.x;
        y = pose.position.y;
        z = pose.position.z;
        base_.legs[i]->joint_chain[j]->setTranslation(x, y, z);
      } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "Exception occured");
      }
    }

    RCLCPP_INFO(get_logger(), "Got links %d for %s", curr_links.size(), links_map[i].c_str());

  }

  /*joint_names_ = champ::URDF::getJointNames(nh); CHAMP*/
  /*joint_names_ = {
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
    "rh_lower_leg_joint"};*/

  /*joint_names_ = champ::URDF::getJointNames(nh); SPOT*/
  joint_names_ = {
    "front_left_hip_x",
    "front_left_hip_y",
    "front_left_knee",
    "front_right_hip_x",
    "front_right_hip_y",
    "front_right_knee",
    "rear_left_hip_x",
    "rear_left_hip_y",
    "rear_left_knee",
    "rear_right_hip_x",
    "rear_right_hip_y",
    "rear_right_knee"};

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000 / loop_rate)),
    std::bind(&QuadrupedController::controlLoop, this));

  req_pose_.position.z = gait_config_.nominal_height;

  RCLCPP_INFO(get_logger(), "Creating .. ");
}

void QuadrupedController::controlLoop()
{
  float target_joint_positions[12];
  geometry::Transformation target_foot_positions[4];
  bool foot_contacts[4];

  body_controller_.poseCommand(target_foot_positions, req_pose_);
  leg_controller_.velocityCommand(
    target_foot_positions, req_vel_,
    rosTimeToChampTime(this->now()));
  kinematics_.inverse(target_joint_positions, target_foot_positions);

  publishFootContacts(foot_contacts);
  publishJoints(target_joint_positions);

  //RCLCPP_DEBUG(get_logger(), "Executing control loop.. ");
}

void QuadrupedController::cmdVelCallback(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
  req_vel_.linear.x = msg->linear.x;
  req_vel_.linear.y = msg->linear.y;
  req_vel_.angular.z = msg->angular.z;

  RCLCPP_DEBUG(get_logger(), "Got velocity commands.. ");
}

void QuadrupedController::cmdPoseCallback(const geometry_msgs::msg::Pose::ConstSharedPtr msg)
{
  tf2::Quaternion quat(
    msg->orientation.x,
    msg->orientation.y,
    msg->orientation.z,
    msg->orientation.w);
  tf2::Matrix3x3 m(quat);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  req_pose_.orientation.roll = roll;
  req_pose_.orientation.pitch = pitch;
  req_pose_.orientation.yaw = yaw;

  req_pose_.position.x = msg->position.x;
  req_pose_.position.y = msg->position.y;
  req_pose_.position.z = msg->position.z + gait_config_.nominal_height;

  RCLCPP_DEBUG(get_logger(), "Got go to pose command.. ");

}

void QuadrupedController::publishJoints(float target_joints[12])
{
  if (publish_joint_control_) {
    trajectory_msgs::msg::JointTrajectory joints_cmd_msg;
    joints_cmd_msg.header.stamp = this->now();
    joints_cmd_msg.joint_names = joint_names_;

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.resize(12);

    point.time_from_start = rclcpp::Duration::from_seconds(0);
    for (size_t i = 0; i < 12; i++) {
      point.positions[i] = target_joints[i];
    }

    joints_cmd_msg.points.push_back(point);
    joint_commands_publisher_->publish(joints_cmd_msg);
  }

  if (publish_joint_states_ && !in_gazebo_) {
    sensor_msgs::msg::JointState joints_msg;

    joints_msg.header.stamp = this->now();
    joints_msg.name.resize(joint_names_.size());
    joints_msg.position.resize(joint_names_.size());
    joints_msg.name = joint_names_;

    for (size_t i = 0; i < joint_names_.size(); ++i) {
      joints_msg.position[i] = target_joints[i];
    }

    joint_states_publisher_->publish(joints_msg);
  }
}

void QuadrupedController::publishFootContacts(bool foot_contacts[4])
{
  if (publish_foot_contacts_ && !in_gazebo_) {
    champ_msgs::msg::ContactsStamped contacts_msg;
    contacts_msg.header.stamp = this->now();
    contacts_msg.contacts.resize(4);

    for (size_t i = 0; i < 4; i++) {
      //This is only published when there's no feedback on the robot
      //that a leg is in contact with the ground
      //For such cases, we use the stance phase in the gait for foot contacts
      contacts_msg.contacts[i] = base_.legs[i]->gait_phase();
    }

    foot_contacts_publisher_->publish(contacts_msg);
  }
}


int main(int argc, char const * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<QuadrupedController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
