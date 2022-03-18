/*
 * Copyright 2021 Fetullah Atas, Norwegian University of Life Sciences

 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "ros2_full_sensor_suite/sensor_base_attacher_plugin.hpp"
#include <gazebo_ros/node.hpp>

#include <iostream>

namespace gazebo
{
  SensorBaseAttacherPlugin::SensorBaseAttacherPlugin()
  {
  }

  SensorBaseAttacherPlugin::~SensorBaseAttacherPlugin()
  {
  }

  void SensorBaseAttacherPlugin::Load(gazebo::physics::WorldPtr _parent, sdf::ElementPtr _sdf)
  {
    // Get model and world references
    world_ = _parent;
    physics_ = world_->Physics();

    // Set up ROS node and subscribers and publishers
    ros_node_ = gazebo_ros::Node::Get(_sdf);
    RCLCPP_INFO(ros_node_->get_logger(), "Loading %s", ros_node_->get_name());

    if (_sdf->HasElement("robot_model_name")) {
      robot_model_name_ = _sdf->GetElement("robot_model_name")->GetValue()->GetAsString();
    }
    if (_sdf->HasElement("robot_base_link_name")) {
      robot_base_link_name_ = _sdf->GetElement("robot_base_link_name")->GetValue()->GetAsString();
    }
    if (_sdf->HasElement("sensor_suite_model_name")) {
      sensor_suite_model_name_ =
        _sdf->GetElement("sensor_suite_model_name")->GetValue()->GetAsString();
    }
    if (_sdf->HasElement("sensor_suite_base_link_name")) {
      sensor_suite_base_link_name_ =
        _sdf->GetElement("sensor_suite_base_link_name")->GetValue()->GetAsString();
    }

    if (_sdf->HasElement("pose")) {
      pose_ = _sdf->Get<ignition::math::Pose3d>("pose");
    } else {
      RCLCPP_INFO(ros_node_->get_logger(), "  plugin missing <pose>, defaults to 0s");
      pose_ = ignition::math::Pose3d(0, 0, 0, 0, 0, 0);
    }

    if (robot_model_name_.empty() || sensor_suite_model_name_.empty()) {
      RCLCPP_ERROR(
        ros_node_->get_logger(),
        "Cannot attach sensor suite to robot model, Empty robot model or sensor suite name");
    }
    if (robot_base_link_name_.empty() || sensor_suite_base_link_name_.empty()) {
      RCLCPP_ERROR(
        ros_node_->get_logger(),
        "Cannot attach sensor suite to robot model, Empty robot base link name");
    }

    RCLCPP_INFO(
      ros_node_->get_logger(), "robot_model_name is set to %s",
      robot_model_name_.c_str());
    RCLCPP_INFO(
      ros_node_->get_logger(), "robot_base_link_name is set to %s",
      robot_base_link_name_.c_str());
    RCLCPP_INFO(
      ros_node_->get_logger(), "sensor_suite_model_name is set to %s",
      sensor_suite_model_name_.c_str());
    RCLCPP_INFO(
      ros_node_->get_logger(), "sensor_suite_base_link_name is set to %s",
      sensor_suite_base_link_name_.c_str());

    std::thread t(&SensorBaseAttacherPlugin::Attach, this);
    t.detach();

    // Hook into simulation update loop
    /*update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&SensorBaseAttacherPlugin::Update, this));*/
  }

  void SensorBaseAttacherPlugin::Update()
  {
    RCLCPP_ERROR(ros_node_->get_logger(), "SensorBaseAttacherPlugin::Update Not Implemented");
  }

  void SensorBaseAttacherPlugin::Attach()
  {

    fixedJoint j;
    j.model1 = robot_model_name_;
    j.link1 = robot_base_link_name_;
    j.model2 = sensor_suite_model_name_;
    j.link2 = sensor_suite_base_link_name_;

    gazebo::physics::BasePtr b1 = NULL;
    gazebo::physics::BasePtr b2 = NULL;

    while (!b1 || !b2) {
      b1 = world_->ModelByName(robot_model_name_);
      b2 = world_->ModelByName(sensor_suite_model_name_);

      if (b1 == NULL) {
        RCLCPP_ERROR(ros_node_->get_logger(), "%s model was not found", robot_model_name_.c_str());
        RCLCPP_ERROR(ros_node_->get_logger(), "Waiting and trying again");

      }

      if (b2 == NULL) {
        RCLCPP_ERROR(
          ros_node_->get_logger(), "%s model was not found", sensor_suite_model_name_.c_str());
        RCLCPP_ERROR(ros_node_->get_logger(), "Waiting and trying again");
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));

    }
    RCLCPP_INFO(ros_node_->get_logger(), "Found given model names, proceeding to attach them");

    gazebo::physics::ModelPtr m1(dynamic_cast<gazebo::physics::Model *>(b1.get()));
    j.m1 = m1;
    gazebo::physics::ModelPtr m2(dynamic_cast<gazebo::physics::Model *>(b2.get()));
    j.m2 = m2;

    gazebo::physics::LinkPtr l1 = m1->GetLink(robot_base_link_name_);
    if (l1 == NULL) {
      RCLCPP_ERROR(ros_node_->get_logger(), "%s was not found", robot_base_link_name_.c_str());
      return;
    }
    j.l1 = l1;


    gazebo::physics::LinkPtr l2 = m2->GetLink(sensor_suite_base_link_name_);
    if (l2 == NULL) {
      RCLCPP_ERROR(
        ros_node_->get_logger(), " %s was not found", sensor_suite_base_link_name_.c_str());
      return;
    }
    j.l2 = l2;

    // Attch sensor suit base link to robot models base link
    j.joint = physics_->CreateJoint("fixed", m1);
    j.joint->Attach(l1, l2);
    j.joint->Load(l1, l2, pose_);
    j.joint->SetModel(m2);
    j.joint->Init();
    RCLCPP_INFO(
      ros_node_->get_logger(), "Attached %s to %s",
      sensor_suite_model_name_.c_str(), robot_model_name_.c_str());
    RCLCPP_INFO(
      ros_node_->get_logger(), "Attached %s ", pose_);
  }
}  // namespace gazebo
GZ_REGISTER_WORLD_PLUGIN(gazebo::SensorBaseAttacherPlugin)
