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

#include <state_estimation.hpp>

champ::Odometry::Time rosTimeToChampTime(const rclcpp::Time & time)
{
  return time.nanoseconds() / 1000ul;
}

StateEstimation::StateEstimation()
: Node("state_estimation_node"),
  tf_broadcaster_(this),
  odometry_(base_, rosTimeToChampTime(this->now()))
{

  joint_states_subscriber_.subscribe(this, "joint_states", rmw_qos_profile_sensor_data);
  foot_contacts_subscriber_.subscribe(this, "foot_contacts", rmw_qos_profile_sensor_data);

  sync.reset(
    new Sync(
      SyncPolicy(10),
      joint_states_subscriber_,
      foot_contacts_subscriber_));

  sync->registerCallback(
    std::bind(
      &StateEstimation::synchronized_callback, this,
      std::placeholders::_1,
      std::placeholders::_2));

  auto custom_qos = rclcpp::QoS(1);

  footprint_to_odom_publisher_ =
    this->create_publisher<nav_msgs::msg::Odometry>("odom/raw", custom_qos);

  base_to_footprint_publisher_ =
    this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "base_to_footprint_pose",
    custom_qos);
  foot_publisher_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("foot", custom_qos);

  this->declare_parameter("base", "base_link");
  this->declare_parameter("gait.odom_scaler", 0.9);
  this->declare_parameter("orientation_from_imu", false);

  this->get_parameter("base", base_name_);
  this->get_parameter("gait.odom_scaler", gait_config_.odom_scaler);
  this->get_parameter("orientation_from_imu", orientation_from_imu_);


  if (orientation_from_imu_) {
    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu/data",
      rclcpp::SystemDefaultsQoS(),
      std::bind(&StateEstimation::imu_callback, this, std::placeholders::_1));
  }

  base_.setGaitConfig(gait_config_);

  /*champ::URDF::loadFromServer(base_, nh);*/
  parameter_client_node_ = std::make_shared<rclcpp::Node>(
    "robot_state_publisher_parameter_client_node");

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

  // read links names from yaml
  std::vector<std::string> links_map;
  links_map.push_back("left_front_links");
  links_map.push_back("left_hind_links");
  links_map.push_back("right_front_links");
  links_map.push_back("right_hind_links");

  for (int i = 0; i < 4; i++) {
    //fillLeg(base_.legs[i], nh, model, links_map[i]);
    this->declare_parameter(links_map[i]);
    rclcpp::Parameter curr_links_param(links_map[i], std::vector<std::string>({}));
    this->get_parameter(links_map[i], curr_links_param);
    std::vector<std::string> curr_links = curr_links_param.as_string_array();

    RCLCPP_INFO(get_logger(), "Got links %d for %s", curr_links.size(), links_map[i].c_str());

    for (int j = 3; j > -1; j--) {
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
    }
  }

  // read joint names from yaml
  std::vector<std::string> joints_map;
  joints_map.push_back("left_front_joints");
  joints_map.push_back("left_hind_joints");
  joints_map.push_back("right_front_joints");
  joints_map.push_back("right_hind_joints");
  for (int i = 0; i < 4; i++) {
    this->declare_parameter(joints_map[i]);
    rclcpp::Parameter curr_joints_param(joints_map[i], std::vector<std::string>({}));
    this->get_parameter(joints_map[i], curr_joints_param);

    RCLCPP_INFO(get_logger(), "joints for %s are listed below", joints_map[i].c_str());
    for (auto && curr_joint : curr_joints_param.as_string_array()) {
      joint_names_.push_back(curr_joint);
      RCLCPP_INFO(get_logger(), "%s", curr_joint.c_str());
    }
  }

  node_namespace_ = this->get_namespace();

  if (node_namespace_.length() > 1) {
    node_namespace_.replace(0, 1, "");
    node_namespace_.push_back('/');
  } else {
    node_namespace_ = "";
  }

  odom_frame_ = node_namespace_ + "odom";
  base_footprint_frame_ = node_namespace_ + "base_footprint";
  base_link_frame_ = node_namespace_ + base_name_;

  odom_data_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&StateEstimation::publishFootprintToOdom, this));

  base_pose_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&StateEstimation::publishBaseToFootprint, this));

  last_vel_time_ = this->now();
  last_sync_time_ = this->now();
}

void StateEstimation::synchronized_callback(
  const sensor_msgs::msg::JointState::ConstSharedPtr & joints_msg,
  const champ_msgs::msg::ContactsStamped::ConstSharedPtr & contacts_msg)
{
  last_sync_time_ = this->now();

  float current_joint_positions[12];

  for (size_t i = 0; i < joints_msg->name.size(); i++) {
    std::vector<std::string>::iterator itr = std::find(
      joint_names_.begin(),
      joint_names_.end(), joints_msg->name[i]);

    int index = std::distance(joint_names_.begin(), itr);
    current_joint_positions[index] = joints_msg->position[i];
  }

  base_.updateJointPositions(current_joint_positions);

  for (size_t i = 0; i < 4; i++) {
    base_.legs[i]->in_contact(contacts_msg->contacts[i]);
  }
}

void StateEstimation::imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  last_imu_ = *msg;
}

void StateEstimation::publishFootprintToOdom()
{
  odometry_.getVelocities(current_velocities_, rosTimeToChampTime(this->now()));

  rclcpp::Time current_time = this->now();

  double vel_dt = (current_time - last_vel_time_).seconds();
  last_vel_time_ = current_time;

  //rotate in the z axis
  //https://en.wikipedia.org/wiki/Rotation_matrix
  double delta_heading = current_velocities_.angular.z * vel_dt;
  double delta_x =
    (current_velocities_.linear.x * cos(heading_) - current_velocities_.linear.y * sin(heading_)) *
    vel_dt;                                                                                                                  //m
  double delta_y =
    (current_velocities_.linear.x * sin(heading_) + current_velocities_.linear.y * cos(heading_)) *
    vel_dt;                                                                                                                  //m

  //calculate current position of the robot
  x_pos_ += delta_x;
  y_pos_ += delta_y;
  heading_ += delta_heading;

  //calculate robot's heading_ in quaternion angle
  tf2::Quaternion odom_quat;
  odom_quat.setRPY(0, 0, heading_);

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = odom_frame_;
  odom.child_frame_id = base_footprint_frame_;

  //robot's position in x,y, and z
  odom.pose.pose.position.x = x_pos_;
  odom.pose.pose.position.y = y_pos_;
  odom.pose.pose.position.z = 0.0;
  //robot's heading_ in quaternion
  odom.pose.pose.orientation.x = odom_quat.x();
  odom.pose.pose.orientation.y = odom_quat.y();
  odom.pose.pose.orientation.z = odom_quat.z();
  odom.pose.pose.orientation.w = odom_quat.w();
  odom.pose.covariance[0] = 0.25;
  odom.pose.covariance[7] = 0.25;
  odom.pose.covariance[35] = 0.017;

  odom.twist.twist.linear.x = current_velocities_.linear.x;
  odom.twist.twist.linear.y = current_velocities_.linear.y;
  odom.twist.twist.linear.z = 0.0;

  odom.twist.twist.angular.x = 0.0;
  odom.twist.twist.angular.y = 0.0;
  odom.twist.twist.angular.z = current_velocities_.angular.z;

  odom.twist.covariance[0] = 0.3;
  odom.twist.covariance[7] = 0.3;
  odom.twist.covariance[35] = 0.017;

  // WARN THIS NEEDS TO BE TESTED
  // create a tf as well from acquired footprint to odom
  geometry_msgs::msg::TransformStamped tf_footprint_to_odom;
  tf_footprint_to_odom.header.frame_id = odom_frame_;
  tf_footprint_to_odom.child_frame_id = base_footprint_frame_;
  tf_footprint_to_odom.header.stamp = this->now();
  tf_footprint_to_odom.transform.translation.x = odom.pose.pose.position.x;
  tf_footprint_to_odom.transform.translation.y = odom.pose.pose.position.y;
  tf_footprint_to_odom.transform.translation.z = odom.pose.pose.position.z;
  tf_footprint_to_odom.transform.rotation = odom.pose.pose.orientation;
  tf_broadcaster_.sendTransform(tf_footprint_to_odom);

  footprint_to_odom_publisher_->publish(odom);
}

visualization_msgs::msg::Marker StateEstimation::createMarker(
  geometry::Transformation foot_pos, int id,
  std::string frame_id)
{
  visualization_msgs::msg::Marker foot_marker;

  foot_marker.header.frame_id = frame_id;

  foot_marker.type = visualization_msgs::msg::Marker::SPHERE;
  foot_marker.action = visualization_msgs::msg::Marker::ADD;
  foot_marker.id = id;

  foot_marker.pose.position.x = foot_pos.X();
  foot_marker.pose.position.y = foot_pos.Y();
  foot_marker.pose.position.z = foot_pos.Z();

  foot_marker.pose.orientation.x = 0.0;
  foot_marker.pose.orientation.y = 0.0;
  foot_marker.pose.orientation.z = 0.0;
  foot_marker.pose.orientation.w = 1.0;

  foot_marker.scale.x = 0.025;
  foot_marker.scale.y = 0.025;
  foot_marker.scale.z = 0.025;

  foot_marker.color.r = 0.780;
  foot_marker.color.g = 0.082;
  foot_marker.color.b = 0.521;
  foot_marker.color.a = 0.5;

  return foot_marker;
}

void StateEstimation::publishBaseToFootprint()
{
  base_.getFootPositions(current_foot_positions_);

  visualization_msgs::msg::MarkerArray marker_array;
  float robot_height = 0.0, all_height = 0.0;
  int foot_in_contact = 0;
  geometry::Transformation touching_feet[4];
  bool no_contact = false;

  for (size_t i = 0; i < 4; i++) {
    marker_array.markers.push_back(createMarker(current_foot_positions_[i], i, base_link_frame_));
    if (base_.legs[i]->in_contact()) {
      robot_height += current_foot_positions_[i].Z();
      touching_feet[foot_in_contact] = current_foot_positions_[i];
      foot_in_contact++;
    }
    all_height += current_foot_positions_[i].Z();
  }

  if (foot_in_contact == 0) {
    no_contact = true;
    robot_height = all_height;
    foot_in_contact = 4;
    for (size_t i = 0; i < 4; ++i) {
      touching_feet[i] = current_foot_positions_[i];
    }
  }

  if (foot_publisher_->get_subscription_count()) {
    foot_publisher_->publish(marker_array);
  }

  tf2::Vector3 x_axis(1, 0, 0);
  tf2::Vector3 y_axis(0, 1, 0);
  tf2::Vector3 z_axis(0, 0, 1);

  // if the IMU provides good orientation estimates, these can be used to
  // greatly improve body orientation; IMUs in Gazebo provide even non-noisy
  // orientation measurements!
  tf2::Matrix3x3 imu_rotation;
  if (orientation_from_imu_ && last_imu_.header.stamp.sec > 0) {
    tf2::Quaternion imu_orientation(
      last_imu_.orientation.x,
      last_imu_.orientation.y,
      last_imu_.orientation.z,
      last_imu_.orientation.w);
    imu_rotation.setRotation(imu_orientation);
  } else {
    imu_rotation.setIdentity();
  }

  // handle the orientation estimation based on the number of touching legs
  if (foot_in_contact >= 3 && !no_contact) {
    // 3 or 4 legs touching. 3 points are enough to form a plane, so we choose
    // any 3 touching legs and create a plane from them

    // create two vectors in base_footprint plane
    x_axis = tf2::Vector3(
      touching_feet[0].X() - touching_feet[2].X(),
      touching_feet[0].Y() - touching_feet[2].Y(),
      touching_feet[0].Z() - touching_feet[2].Z());
    x_axis.normalize();

    y_axis = tf2::Vector3(
      touching_feet[1].X() - touching_feet[2].X(),
      touching_feet[1].Y() - touching_feet[2].Y(),
      touching_feet[1].Z() - touching_feet[2].Z());
    y_axis.normalize();

    // compute normal vector of the plane
    z_axis = x_axis.cross(y_axis);
    z_axis.normalize();

    // we don't know which 3 feet were chosen, so it might happen the normal points downwards
    if (z_axis.dot(tf2::Vector3(0, 0, 1)) < 0) {
      z_axis = -z_axis;
    }

    // project 0,1,0 base_link axis to the plane defined by the normal
    y_axis = (tf2::Vector3(0, 1, 0) - (tf2::Vector3(0, 1, 0).dot(z_axis) * z_axis)).normalized();
    // and find the last vector which just has to be perpendicular to y and z
    x_axis = y_axis.cross(z_axis);
  } else if (foot_in_contact == 2) {
    if ((base_.legs[0]->in_contact() && base_.legs[2]->in_contact()) ||
      (base_.legs[1]->in_contact() && base_.legs[3]->in_contact()))
    {
      // both left or both right legs are touching... let them define the x axis
      x_axis = tf2::Vector3(
        touching_feet[0].X() - touching_feet[1].X(),
        touching_feet[0].Y() - touching_feet[1].Y(),
        touching_feet[0].Z() - touching_feet[1].Z());
      x_axis.normalize();

      // get Z from IMU as we do not have enough contact points to define a plane
      z_axis = imu_rotation.inverse() * z_axis;
      y_axis = z_axis.cross(x_axis);
      // and find the last vector which just has to be perpendicular to y and z
      x_axis = y_axis.cross(z_axis);
    } else if ((base_.legs[0]->in_contact() && base_.legs[1]->in_contact()) ||
      (base_.legs[2]->in_contact() && base_.legs[3]->in_contact()))
    {
      // both front or both hind legs are touching... let them define the y axis
      y_axis = tf2::Vector3(
        touching_feet[0].X() - touching_feet[1].X(),
        touching_feet[0].Y() - touching_feet[1].Y(),
        touching_feet[0].Z() - touching_feet[1].Z());
      y_axis.normalize();

      // get Z from IMU as we do not have enough contact points to define a plane
      z_axis = imu_rotation.inverse() * z_axis;
      x_axis = y_axis.cross(z_axis);
      // and find the last vector which just has to be perpendicular to x and z
      y_axis = z_axis.cross(x_axis);
    } else {
      // diagonal legs touching... axis1 is the line going through both touching
      // legs. axis2 is perpendicular to axis1 and z axis (from IMU)... then we
      // just rotate axis1 and axis2 to form a coordinate system
      tf2::Vector3 axis1(touching_feet[0].X() - touching_feet[1].X(),
        touching_feet[0].Y() - touching_feet[1].Y(),
        touching_feet[0].Z() - touching_feet[1].Z());
      axis1.normalize();

      // get Z from IMU as we do not have enough contact points to define a plane
      z_axis = imu_rotation.inverse() * z_axis;
      auto axis2 = z_axis.cross(axis1);
      z_axis = axis1.cross(axis2);

      // project base_link 1,0,0 axis along the computed plane normal
      x_axis = (x_axis - (x_axis.dot(z_axis) * z_axis)).normalized();
      // and find the last vector which just has to be perpendicular to x and z
      y_axis = z_axis.cross(x_axis);
    }
  } else if (foot_in_contact == 1 || no_contact) {
    // Zero or one feet in contact... There isn't much to do, so just take Z from IMU
    z_axis = imu_rotation.inverse() * z_axis;

    // project base_link 1,0,0 axis along the computed plane normal
    x_axis = (x_axis - (x_axis.dot(z_axis) * z_axis)).normalized();
    // and find the last vector which just has to be perpendicular to x and z
    y_axis = z_axis.cross(x_axis);
  }

  tf2::Matrix3x3 rotationMatrix(
    x_axis.x(), y_axis.x(), z_axis.x(),
    x_axis.y(), y_axis.y(), z_axis.y(),
    x_axis.z(), y_axis.z(), z_axis.z());

  tf2::Quaternion quaternion;
  rotationMatrix.getRotation(quaternion);
  quaternion.normalize();


  geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
  pose_msg.header.frame_id = base_footprint_frame_;
  pose_msg.header.stamp = this->now();

  pose_msg.pose.covariance[0] = 0.001;
  pose_msg.pose.covariance[7] = 0.001;
  pose_msg.pose.covariance[14] = 0.001;
  pose_msg.pose.covariance[21] = 0.0001;
  pose_msg.pose.covariance[28] = 0.0001;
  pose_msg.pose.covariance[35] = 0.017;

  pose_msg.pose.pose.position.x = 0.0;
  pose_msg.pose.pose.position.y = 0.0;
  pose_msg.pose.pose.position.z = -(robot_height / (float)foot_in_contact);

  /*pose_msg.pose.pose.orientation.x = quaternion.x();
  pose_msg.pose.pose.orientation.y = quaternion.y();
  pose_msg.pose.pose.orientation.z = quaternion.z();
  pose_msg.pose.pose.orientation.w = -quaternion.w();*/
  pose_msg.pose.pose.orientation.x = 0;
  pose_msg.pose.pose.orientation.y = 0;
  pose_msg.pose.pose.orientation.z = 0;
  pose_msg.pose.pose.orientation.w = 1.0;

  base_to_footprint_publisher_->publish(pose_msg);
}

int main(int argc, char const * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StateEstimation>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
