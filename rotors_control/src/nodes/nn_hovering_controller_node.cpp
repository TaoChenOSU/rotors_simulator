/*
** This is a neural network controller node that evaluates
** the neural network with a set of weights and outputs the
** control commands to hover.
** This is an experiment.
** Tao Chen, 05/25/2018
*/

#include <ros/ros.h>
#include <mav_msgs/default_topics.h>
#include "nn_hovering_controller_node.h"

namespace rotors_control{
  NNHoveringControllerNode::NNHoveringControllerNode(
          const ros::NodeHandle& nh, const ros::NodeHandle& private_nh, const char* model)
          : nh_(nh), private_nh_(private_nh)
  {
    odometry_sub_ = nh_.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
                                &NNHoveringControllerNode::OdometryCallback, this);

    trajectory_sub_ = nh_.subscribe(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 1,
      &NNHoveringControllerNode::TrajectoryCallback, this);

    motor_velocity_reference_pub_ = nh_.advertise<mav_msgs::Actuators>(
                                mav_msgs::default_topics::COMMAND_ACTUATORS, 1);

    command_timer_ = nh_.createTimer(ros::Duration(0),
                                &NNHoveringControllerNode::TimedCommandCallback, this, true, false);

    nn_hovering_controller_.SetModelPath(model);
    nn_hovering_controller_.init();
  }

  NNHoveringControllerNode::~NNHoveringControllerNode() {}

  void NNHoveringControllerNode::TimedCommandCallback(const ros::TimerEvent& e) {

    if (commands_.empty()) {
      ROS_WARN("No more points, command wait time and the # of commands don't match.\n");
      return;
    }

    const mav_msgs::EigenTrajectoryPoint eigen_reference = commands_.front();
    ROS_INFO("Executing the point:\n %s. \n", eigen_reference.toString().c_str());
    nn_hovering_controller_.SetTrajectoryPoint(eigen_reference);
    commands_.pop_front();
    command_timer_.stop();
    if (!command_waiting_times_.empty()) {
      command_timer_.setPeriod(command_waiting_times_.front());
      command_waiting_times_.pop_front();
      command_timer_.start();
    }
  }

  void NNHoveringControllerNode::TrajectoryCallback(
      const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg) {

    command_timer_.stop();
    commands_.clear();
    command_waiting_times_.clear();

    const size_t n_commands = msg->points.size();
    if (n_commands < 1) {
      ROS_WARN_STREAM("Got MultiDOFJointTrajectory message, but message has no points.");
      return;
    }
    ROS_INFO("Received a new trajectory. This trajectory has %zu points.\n", n_commands);

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

    ROS_INFO("Executing the point:\n%s. \n", commands_.front().toString().c_str());
    nn_hovering_controller_.SetTrajectoryPoint(commands_.front());
    commands_.pop_front();

    if (n_commands > 1) {
      command_timer_.setPeriod(command_waiting_times_.front());
      command_waiting_times_.pop_front();
      command_timer_.start();
    }
  }

  void NNHoveringControllerNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {
    ROS_INFO_ONCE("NNHoveringController got first odometry message.");

    nn_hovering_controller_.SetOdometry(odometry_msg);

    Eigen::VectorXf ref_rotor_velocities;
    nn_hovering_controller_.CalculateRotorVelocities(&ref_rotor_velocities);

    mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);
    actuator_msg->angular_velocities.clear();

    // assuming it's a quadrotor (for now)
    for(int i = 0; i < ref_rotor_velocities.size(); i++){
      actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
    }

    actuator_msg->header.stamp = odometry_msg->header.stamp;
    motor_velocity_reference_pub_.publish(actuator_msg);
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "nn_hovering_controller_node");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  if (argc != 2) {
    ROS_ERROR("Missing argument, possibly the model index");
    exit(1);
  } else {
    ROS_INFO("You have selected to use model %s.\n", argv[1]);
    rotors_control::NNHoveringControllerNode nn_hovering_controller_node(nh, private_nh, argv[1]);
    ros::spin();
  }

  return 0;
}
