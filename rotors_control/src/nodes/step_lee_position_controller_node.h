/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
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

#ifndef ROTORS_CONTROL_LEE_POSITION_CONTROLLER_NODE_H
#define ROTORS_CONTROL_LEE_POSITION_CONTROLLER_NODE_H

#include <thread>
#include <chrono>

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <stdio.h>
#include <random>
#include <math.h>

#include <geometry_msgs/PoseStamped.h>
#include <mav_msgs/Actuators.h>
#include <mav_msgs/AttitudeThrust.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <nav_msgs/Odometry.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include "rotors_control/common.h"
#include "rotors_control/lee_position_controller.h"
#include <rotors_step_simulation_plugin/RequestToTakeNSteps.h>
#include <rotors_step_simulation_plugin/RequestToResetState.h>

#define PI 3.1415926535897

namespace rotors_control {

class LeePositionControllerNode {
 public:
  LeePositionControllerNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
  ~LeePositionControllerNode();

  void InitializeParams();
  void Publish_Command();
  void startRequest();
  void reset();

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  LeePositionController lee_position_controller_;

  std::string namespace_;

  // subscribers
  ros::Subscriber cmd_trajectory_sub_;
  ros::Subscriber cmd_multi_dof_joint_trajectory_sub_;
  ros::Subscriber cmd_pose_sub_;

  // service client
  ros::ServiceClient request_to_take_n_steps_client;
  ros::ServiceClient request_to_reset_client;
  // service instant
  rotors_step_simulation_plugin::RequestToTakeNSteps srv_step;
  // odom message for the controller responsed by gazebo plugin
  nav_msgs::Odometry odom;

  // params for simulation
  int controller_wait_interval;
  int gazebo_step_size;

  int t_max;
  int t_iter;
  int total_time;
  int num_of_trajs;

  mav_msgs::EigenTrajectoryPointDeque commands_;
  std::deque<ros::Duration> command_waiting_times_;
  ros::Timer command_timer_;

  mav_msgs::Actuators actuator_msg;

  void TimedCommandCallback(const ros::TimerEvent& e);

  void MultiDofJointTrajectoryCallback(
      const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& trajectory_reference_msg);

  void CommandPoseCallback(
      const geometry_msgs::PoseStampedConstPtr& pose_msg);

  rotors_step_simulation_plugin::RequestToResetState GetNewStateSrv();

  std::default_random_engine generator;

  double UniformDistribution(double lower, double upper); 

  double NormalDistribution(double mean, double stddev);
};
}

#endif // ROTORS_CONTROL_LEE_POSITION_CONTROLLER_NODE_H
