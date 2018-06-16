/*
** This is a node to generate state randomly based on certain
** distributions to speed up the data collection process on controllers.
*/

#ifndef ROTORS_CONTROL_RANDOM_STATE_GENERATOR_H
#define ROTORS_CONTROL_RANDOM_STATE_GENERATOR_H

#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <Eigen/Core>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/conversions.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <gazebo_msgs/SetModelState.h>

#include <random>
#include <math.h>
#include <string>

#define PI 3.1415926535897

namespace rotors_control {
  class RandomStateGenerator {
    public:
      RandomStateGenerator(const ros::NodeHandle& nh, int num_of_trajs);
      ~RandomStateGenerator();

    private:
      ros::NodeHandle nh_;
      // ros::Publisher state_pub_;
      ros::ServiceClient set_state_srv;
      gazebo_msgs::SetModelState srv;

      ros::Publisher trajectory_pub;
      ros::Timer command_timer_;
      float stddev;
      float mean;

      int counter;
      int num_of_trajs;

      std::default_random_engine generator;

      void WaypointPub();

      // void PublishState(const ros::TimerEvent& e);

      void SetState(const ros::TimerEvent& e);

      void GetNewState(nav_msgs::Odometry& state);

      double NormalDistribution(double mean, double stddev);

      double UniformDistribution(double lower, double upper);
  };
}

#endif
