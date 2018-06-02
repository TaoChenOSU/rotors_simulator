/*
** This is a node to read trajectories produced by the neural net controller
** and pass them through the lee controller to get control outputs. (Dagger)
*/

#ifndef DAGGER_ROUTINE_H
#define DAGGER_ROUTINE_H

#include <ros/ros.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/conversions.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include "rotors_control/lee_position_controller.h"

#include <boost/algorithm/string.hpp>
#include <fstream>

namespace rotors_control {
  class DaggerRoutine {
    public:
      DaggerRoutine(const ros::NodeHandle& nh);
      ~DaggerRoutine();

    private:
      ros::NodeHandle nh_;
      ros::Publisher state_pub_;
      ros::Publisher trajectory_pub_;
      ros::Timer command_timer_;

      std::vector<nav_msgs::Odometry> odometry_msgs;

      int ReadStates();

      // void LogRotorVelocities();

      void WaypointPub();

      void PublishState(const ros::TimerEvent& e);
  };
}
#endif
