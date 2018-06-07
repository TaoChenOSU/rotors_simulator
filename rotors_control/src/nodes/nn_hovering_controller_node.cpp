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
          const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
          : nh_(nh), private_nh_(private_nh)
  {
    odometry_sub_ = nh_.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
                                &NNHoveringControllerNode::OdometryCallback, this);

    motor_velocity_reference_pub_ = nh_.advertise<mav_msgs::Actuators>(
                                mav_msgs::default_topics::COMMAND_ACTUATORS, 1);

    command_timer_ = nh_.createTimer(ros::Duration(0),
                                &NNHoveringControllerNode::TimedCommandCallback, this, true, false);
  }

  NNHoveringControllerNode::~NNHoveringControllerNode() {}

  void NNHoveringControllerNode::TimedCommandCallback(const ros::TimerEvent& e) {}

  void NNHoveringControllerNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {
    ROS_INFO_ONCE("NNHoveringController got first odometry message.");

    ROS_INFO("CONTROLLER_INPUT_STATES: %.16f %.16f %.16f %.16f %.16f %.16f %.16f %.16f %.16f %.16f %.16f %.16f %.16f\n",
                                  odometry_msg->pose.pose.position.x,
                                  odometry_msg->pose.pose.position.y,
                                  odometry_msg->pose.pose.position.z,
                                  odometry_msg->pose.pose.orientation.x,
                                  odometry_msg->pose.pose.orientation.y,
                                  odometry_msg->pose.pose.orientation.z,
                                  odometry_msg->pose.pose.orientation.w,
                                  odometry_msg->twist.twist.linear.x,
                                  odometry_msg->twist.twist.linear.y,
                                  odometry_msg->twist.twist.linear.z,
                                  odometry_msg->twist.twist.angular.x,
                                  odometry_msg->twist.twist.angular.y,
                                  odometry_msg->twist.twist.angular.z);

    nn_hovering_controller_.SetOdometry(odometry_msg);

    Eigen::VectorXf ref_rotor_velocities;
    nn_hovering_controller_.CalculateRotorVelocities(&ref_rotor_velocities);

    mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);
    actuator_msg->angular_velocities.clear();

    // assuming it's a quadrotor (for now)
    for(int i = 0; i < 4; i++){
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
  rotors_control::NNHoveringControllerNode nn_hovering_controller_node(nh, private_nh);

  ros::spin();

  return 0;
}
