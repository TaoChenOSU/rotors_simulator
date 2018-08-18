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

#include <ros/ros.h>
#include <mav_msgs/default_topics.h>

#include "step_lee_position_controller_node.h"

#include "rotors_control/parameters_ros.h"

namespace rotors_control {

LeePositionControllerNode::LeePositionControllerNode(
  const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
  :nh_(nh),
   private_nh_(private_nh){
  InitializeParams();

  cmd_pose_sub_ = nh_.subscribe(
      mav_msgs::default_topics::COMMAND_POSE, 1,
      &LeePositionControllerNode::CommandPoseCallback, this);

  cmd_multi_dof_joint_trajectory_sub_ = nh_.subscribe(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 1,
      &LeePositionControllerNode::MultiDofJointTrajectoryCallback, this);

  command_timer_ = nh_.createTimer(ros::Duration(0), &LeePositionControllerNode::TimedCommandCallback, this,
                                  true, false);

  // just to prevent the controller from crashing
  // we don't need to record data, however, we can
  lee_position_controller_.SetLogDataType("1");

  request_to_take_n_steps_client = nh_.serviceClient<rotors_step_simulation_plugin::RequestToTakeNSteps>(
    "/gazebo_step/take_n_steps");
  request_to_reset_client = nh_.serviceClient<rotors_step_simulation_plugin::RequestToResetState>(
    "/gazebo_step/reset");

  // get params for simulation
  nh_.param("/hummingbird/step_lee_position_controller_node/controller_wait_interval", 
                controller_wait_interval, 10); // default 10ms
  nh_.param("/hummingbird/step_lee_position_controller_node/gazebo_step_size", 
                gazebo_step_size, 2); // default 2 steps per command
  nh_.param("/hummingbird/step_lee_position_controller_node/t_max", 
                t_max, 1000); // default 100 iterations every reset

  // output commands of the controller
  // for quadrotors only (4 motors)
  actuator_msg = mav_msgs::Actuators();

  // initialize command
  LeePositionControllerNode::srv_step.request.step_size = gazebo_step_size;
  LeePositionControllerNode::srv_step.request.motor_speeds = {0.0, 0.0, 0.0, 0.0};

  t_iter = 0;
} 

LeePositionControllerNode::~LeePositionControllerNode() {}

void LeePositionControllerNode::reset() {
  // check if the service exists
  if (!ros::service::exists("/gazebo_step/reset", false)) {
    ROS_INFO_ONCE("Waiting for reset service");
    return;
  }

  rotors_step_simulation_plugin::RequestToResetState srv = GetNewStateSrv();

  if (request_to_reset_client.call(srv)) {
    ROS_INFO("is_reset %d", srv.response.is_reset);
  }

}

void LeePositionControllerNode::startRequest() {
  // check if the service exists
  if (!ros::service::exists("/gazebo_step/take_n_steps", false)) {
    ROS_INFO_ONCE("Waiting for step service");
    return;
  }

  if (request_to_take_n_steps_client.call(srv_step)) {
    odom = srv_step.response.new_state;

    // ROS_INFO("Step took: %d", srv_step.response.step_took);

    nav_msgs::OdometryConstPtr odom_ptr(new nav_msgs::Odometry(odom));

    EigenOdometry odometry;
    eigenOdometryFromMsg(odom_ptr, &odometry);
    lee_position_controller_.SetOdometry(odometry);

    Eigen::VectorXd ref_rotor_velocities;
    lee_position_controller_.CalculateRotorVelocities(&ref_rotor_velocities);

    srv_step.request.step_size = gazebo_step_size;
    srv_step.request.motor_speeds.clear();
    for (int i = 0; i < 4; i++) {
      srv_step.request.motor_speeds.push_back(ref_rotor_velocities[i]);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(controller_wait_interval));
    t_iter++;
    if (t_iter == t_max) {
      reset();
      t_iter = 0;
    }
  }
}

void LeePositionControllerNode::InitializeParams() {

  // Read parameters from rosparam.
  GetRosParameter(private_nh_, "position_gain/x",
                  lee_position_controller_.controller_parameters_.position_gain_.x(),
                  &lee_position_controller_.controller_parameters_.position_gain_.x());
  GetRosParameter(private_nh_, "position_gain/y",
                  lee_position_controller_.controller_parameters_.position_gain_.y(),
                  &lee_position_controller_.controller_parameters_.position_gain_.y());
  GetRosParameter(private_nh_, "position_gain/z",
                  lee_position_controller_.controller_parameters_.position_gain_.z(),
                  &lee_position_controller_.controller_parameters_.position_gain_.z());
  GetRosParameter(private_nh_, "velocity_gain/x",
                  lee_position_controller_.controller_parameters_.velocity_gain_.x(),
                  &lee_position_controller_.controller_parameters_.velocity_gain_.x());
  GetRosParameter(private_nh_, "velocity_gain/y",
                  lee_position_controller_.controller_parameters_.velocity_gain_.y(),
                  &lee_position_controller_.controller_parameters_.velocity_gain_.y());
  GetRosParameter(private_nh_, "velocity_gain/z",
                  lee_position_controller_.controller_parameters_.velocity_gain_.z(),
                  &lee_position_controller_.controller_parameters_.velocity_gain_.z());
  GetRosParameter(private_nh_, "attitude_gain/x",
                  lee_position_controller_.controller_parameters_.attitude_gain_.x(),
                  &lee_position_controller_.controller_parameters_.attitude_gain_.x());
  GetRosParameter(private_nh_, "attitude_gain/y",
                  lee_position_controller_.controller_parameters_.attitude_gain_.y(),
                  &lee_position_controller_.controller_parameters_.attitude_gain_.y());
  GetRosParameter(private_nh_, "attitude_gain/z",
                  lee_position_controller_.controller_parameters_.attitude_gain_.z(),
                  &lee_position_controller_.controller_parameters_.attitude_gain_.z());
  GetRosParameter(private_nh_, "angular_rate_gain/x",
                  lee_position_controller_.controller_parameters_.angular_rate_gain_.x(),
                  &lee_position_controller_.controller_parameters_.angular_rate_gain_.x());
  GetRosParameter(private_nh_, "angular_rate_gain/y",
                  lee_position_controller_.controller_parameters_.angular_rate_gain_.y(),
                  &lee_position_controller_.controller_parameters_.angular_rate_gain_.y());
  GetRosParameter(private_nh_, "angular_rate_gain/z",
                  lee_position_controller_.controller_parameters_.angular_rate_gain_.z(),
                  &lee_position_controller_.controller_parameters_.angular_rate_gain_.z());
  GetVehicleParameters(private_nh_, &lee_position_controller_.vehicle_parameters_);
  lee_position_controller_.InitializeParameters();
}

void LeePositionControllerNode::CommandPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& pose_msg) {
  ROS_INFO("CommandPoseCallback");
  // Clear all pending commands.
  command_timer_.stop();
  commands_.clear();
  command_waiting_times_.clear();

  mav_msgs::EigenTrajectoryPoint eigen_reference;
  mav_msgs::eigenTrajectoryPointFromPoseMsg(*pose_msg, &eigen_reference);
  commands_.push_front(eigen_reference);

  lee_position_controller_.SetTrajectoryPoint(commands_.front());
  commands_.pop_front();
}

void LeePositionControllerNode::MultiDofJointTrajectoryCallback(
    const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg) {
  ROS_INFO("MultiDofJointTrajectoryCallback");
  // Clear all pending commands.
  command_timer_.stop();
  commands_.clear();
  command_waiting_times_.clear();

  const size_t n_commands = msg->points.size();

  if(n_commands < 1){
    ROS_WARN_STREAM("Got MultiDOFJointTrajectory message, but message has no points.");
    return;
  }

  mav_msgs::EigenTrajectoryPoint eigen_reference;
  mav_msgs::eigenTrajectoryPointFromMsg(msg->points.front(), &eigen_reference);
  commands_.push_front(eigen_reference);

  for (size_t i = 1; i < n_commands; ++i) {
    const trajectory_msgs::MultiDOFJointTrajectoryPoint& reference_before = msg->points[i-1];
    const trajectory_msgs::MultiDOFJointTrajectoryPoint& current_reference = msg->points[i];

    mav_msgs::eigenTrajectoryPointFromMsg(current_reference, &eigen_reference);

    commands_.push_back(eigen_reference);
    command_waiting_times_.push_back(current_reference.time_from_start - reference_before.time_from_start);
  }

  // We can trigger the first command immediately.
  lee_position_controller_.SetTrajectoryPoint(commands_.front());
  commands_.pop_front();

  if (n_commands > 1) {
    command_timer_.setPeriod(command_waiting_times_.front());
    command_waiting_times_.pop_front();
    command_timer_.start();
  }
}

void LeePositionControllerNode::TimedCommandCallback(const ros::TimerEvent& e) {

  if(commands_.empty()){
    ROS_WARN("Commands empty, this should not happen here");
    return;
  }

  const mav_msgs::EigenTrajectoryPoint eigen_reference = commands_.front();
  lee_position_controller_.SetTrajectoryPoint(commands_.front());
  commands_.pop_front();
  command_timer_.stop();
  if(!command_waiting_times_.empty()){
    command_timer_.setPeriod(command_waiting_times_.front());
    command_waiting_times_.pop_front();
    command_timer_.start();
  } else {
    // this is added for ending the data collection once the last point is reached
    std_srvs::Empty srv;
    ros::service::call("/gazebo/pause_physics", srv);
  }
}

rotors_step_simulation_plugin::RequestToResetState LeePositionControllerNode::GetNewStateSrv() {
    nav_msgs::Odometry state;
    rotors_step_simulation_plugin::RequestToResetState srv;

    // double normalizedFactor;
    double roll, pitch, yaw;

    state.pose.pose.position.x = NormalDistribution(0.0, 0.8);
    state.pose.pose.position.y = NormalDistribution(0.0, 0.8);
    state.pose.pose.position.z = UniformDistribution(0.0, 2.0);

    // uniform distributed orientation, according to http://www.cognitive-antics.net/uniform-random-orientation/
    roll = UniformDistribution(-PI/12, PI/12);
    pitch = asin((sin(PI/12)-sin(-PI/12))*UniformDistribution(0, 1));
    // yaw = UniformDistribution(-PI/6, PI/6);
    yaw = 0.0;

    // This might not generate uniform distributed orientations
    double cy = cos(yaw/2);
    double sy = sin(yaw/2);
    double cr = cos(roll/2);
    double sr = sin(roll/2);
    double cp = cos(pitch/2);
    double sp = sin(pitch/2);

    state.pose.pose.orientation.x = cy*sr*cp - sy*cr*sp;
    state.pose.pose.orientation.y = cy*cr*sp + sy*sr*cp;
    state.pose.pose.orientation.z = sy*cr*cp - cy*sr*sp;
    state.pose.pose.orientation.w = cy*cr*cp + sy*sr*sp;

    state.twist.twist.linear.x = 0;
    state.twist.twist.linear.y = 0;
    state.twist.twist.linear.z = 0;

    state.twist.twist.angular.x = 0;
    state.twist.twist.angular.y = 0;
    state.twist.twist.angular.z = 0;

    srv.request.robot_name = "hummingbird";
    srv.request.state = state;

    return srv;
  }

  double LeePositionControllerNode::NormalDistribution(double mean, double stddev) {
    std::normal_distribution<double> distribution(mean, stddev);
    return distribution(generator);
  }

  double LeePositionControllerNode::UniformDistribution(double lower, double upper) {
    std::uniform_real_distribution<double> distribution(lower, upper);
    return distribution(generator);
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "lee_position_controller_node");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  rotors_control::LeePositionControllerNode lee_position_controller_node(nh, private_nh);
  while(ros::ok()) {
    ros::spinOnce();
    lee_position_controller_node.startRequest();
  }

  return 0;
}
