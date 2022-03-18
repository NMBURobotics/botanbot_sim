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


#ifndef ROS2_FULL_SENSOR_SUITE__SENSOR_BASE_ATTACHER_PLUGIN_HPP_
#define ROS2_FULL_SENSOR_SUITE__SENSOR_BASE_ATTACHER_PLUGIN_HPP_

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/PhysicsEngine.hh>

#include <rclcpp/rclcpp.hpp>

namespace gazebo
{
  class SensorBaseAttacherPlugin : public gazebo::WorldPlugin
  {
  public:
    SensorBaseAttacherPlugin();
    ~SensorBaseAttacherPlugin();

    void Load(gazebo::physics::WorldPtr _parent, sdf::ElementPtr _sdf);
    void Attach();

  private:
    void Update();

    gazebo::physics::WorldPtr world_;
    gazebo::event::ConnectionPtr update_connection_;
    rclcpp::Node::SharedPtr ros_node_;
    gazebo::physics::PhysicsEnginePtr physics_;

    std::string robot_model_name_;
    std::string robot_base_link_name_;
    std::string sensor_suite_model_name_;
    std::string sensor_suite_base_link_name_;

    ignition::math::Pose3d pose_;

    struct fixedJoint
    {
      std::string model1;
      gazebo::physics::ModelPtr m1;
      std::string link1;
      gazebo::physics::LinkPtr l1;
      std::string model2;
      gazebo::physics::ModelPtr m2;
      std::string link2;
      gazebo::physics::LinkPtr l2;
      gazebo::physics::JointPtr joint;
    };
  };

}  // namespace gazebo

#endif  // ROS2_FULL_SENSOR_SUITE__SENSOR_BASE_ATTACHER_PLUGIN_HPP_
