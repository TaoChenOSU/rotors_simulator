/*
** This is a neural network controller node that evaluates
** the neural network with a set of weights and outputs the
** control commands to hover.
** This is an experiment.
** Tao Chen, 05/25/2018
*/

#ifndef ROTORS_CONTROL_NN_HOVERING_CONTROLLER_NODE_H
#define ROTORS_CONTROL_NN_HOVERING_CONTROLLER_NODE_H

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <stdio.h>

#include <geometry_msgs/PoseStamped.h>
#include <mav_msgs/Actuators.h>
#include <mav_msgs/AttitudeThrust.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <nav_msgs/Odometry.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include "rotors_control/common.h"
#include "rotors_control/nn_hovering_controller.h"

namespace rotors_control{

  class NNHoveringControllerNode {
    public:
      NNHoveringControllerNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
      ~NNHoveringControllerNode();

    private:
      ros::NodeHandle nh_;
      ros::NodeHandle private_nh_;

      NNHoveringController nn_hovering_controller_;

      std::string namespace_;

      // subscribers
      ros::Subscriber odometry_sub_;

      ros::Publisher motor_velocity_reference_pub_;

      std::deque<ros::Duration> command_waiting_times_;
      ros::Timer command_timer_;

      void TimedCommandCallback(const ros::TimerEvent& e);
      void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);
    };
}
#endif
