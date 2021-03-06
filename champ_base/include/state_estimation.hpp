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

#ifndef STATE_ESTIMATION_H
#define STATE_ESTIMATION_H

#include "rclcpp/rclcpp.hpp"
#include <champ_msgs/msg/contacts_stamped.hpp>
#include <champ/odometry/odometry.h>
//#include <champ/utils/urdf_loader.h>

#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/msg/imu.hpp>
#include <champ_msgs/msg/imu.hpp>
#include <champ/quadruped_base/quadruped_components.h>
#include <champ/geometry/geometry.h>
#include <champ/quadruped_base/quadruped_base.h>
#include <urdf/model.h>


class StateEstimation : public rclcpp::Node
{
  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::JointState, champ_msgs::msg::ContactsStamped> SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Sync;
  std::shared_ptr<Sync> sync;

  message_filters::Subscriber<sensor_msgs::msg::JointState> joint_states_subscriber_;
  message_filters::Subscriber<champ_msgs::msg::ContactsStamped> foot_contacts_subscriber_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr footprint_to_odom_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    base_to_footprint_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr foot_publisher_;

  rclcpp::TimerBase::SharedPtr odom_data_timer_;
  rclcpp::TimerBase::SharedPtr base_pose_timer_;

  champ::Velocities current_velocities_;
  geometry::Transformation current_foot_positions_[4];
  geometry::Transformation target_foot_positions_[4];

  float x_pos_;
  float y_pos_;
  float heading_;
  rclcpp::Time last_vel_time_;
  rclcpp::Time last_sync_time_;
  sensor_msgs::msg::Imu last_imu_;

  champ::GaitConfig gait_config_;

  champ::QuadrupedBase base_;
  champ::Odometry odometry_;

  std::vector<std::string> joint_names_;
  std::string base_name_;
  std::string node_namespace_;
  std::string odom_frame_;
  std::string base_footprint_frame_;
  std::string base_link_frame_;
  bool orientation_from_imu_;

  // TF broadcaster
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  rclcpp::Node::SharedPtr parameter_client_node_;

  void publishFootprintToOdom();
  void publishBaseToFootprint();
  void synchronized_callback(
    const sensor_msgs::msg::JointState::ConstSharedPtr & j,
    const champ_msgs::msg::ContactsStamped::ConstSharedPtr & c);
  void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr imu);

  visualization_msgs::msg::Marker createMarker(
    geometry::Transformation foot_pos, int id,
    std::string frame_id);

  void getPose(urdf::Pose * pose, std::string ref_link, std::string end_link, urdf::Model & model)
  {
    urdf::LinkConstSharedPtr ref_link_ptr = model.getLink(ref_link);

    std::string current_parent_name = end_link;
    urdf::LinkConstSharedPtr prev_link = model.getLink(current_parent_name);

    while (ref_link_ptr->name != current_parent_name) {
      urdf::LinkConstSharedPtr current_link = model.getLink(current_parent_name);
      urdf::Pose current_pose = current_link->parent_joint->parent_to_joint_origin_transform;

      current_parent_name = current_link->getParent()->name;
      prev_link = model.getLink(current_parent_name);
      pose->position.x += current_pose.position.x;
      pose->position.y += current_pose.position.y;
      pose->position.z += current_pose.position.z;
    }
  }

public:
  StateEstimation();
};

#endif
