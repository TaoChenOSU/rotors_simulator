/*
** This is a node to generate state randomly based on certain
** distributions to speed up the data collection process on controllers.
*/

#include "random_state_generator.h"

namespace rotors_control {
  RandomStateGenerator::RandomStateGenerator(const ros::NodeHandle& nh)
    : nh_(nh) {

    state_pub_ = nh_.advertise<nav_msgs::Odometry>(
      mav_msgs::default_topics::ODOMETRY, 1);

    trajectory_pub = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);

    WaypointPub();

    command_timer_ = nh_.createTimer(ros::Duration(0.01), &RandomStateGenerator::PublishState, this);
  }

  RandomStateGenerator::~RandomStateGenerator() {}

  void RandomStateGenerator::WaypointPub() {
    trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
    trajectory_msg.header.stamp = ros::Time::now();

    Eigen::Vector3d desired_position(0.0, 0.0, 1.0);
    double desired_yaw = 0.0;

    mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
      desired_position, desired_yaw, &trajectory_msg);

    ros::Duration(5.0).sleep();

    trajectory_pub.publish(trajectory_msg);
    ROS_INFO("Way point published: [%f, %f, %f]",
            desired_position.x(), desired_position.y(), desired_position.z());
  }

  void RandomStateGenerator::PublishState(const ros::TimerEvent& e) {
    nav_msgs::Odometry state;
    GetNewState(state);
    state_pub_.publish(state);
  }

  // only positions vary.
  // velocities and orientations are collected through simulation (Dagger)
  void RandomStateGenerator::GetNewState(nav_msgs::Odometry& state) {
    double normalizedFactor;

    state.pose.pose.position.x = NormalDistribution(0.0, 0.2);
    state.pose.pose.position.y = NormalDistribution(0.0, 0.2);
    state.pose.pose.position.z = UniformDistribution(0.0, 1.2);

    state.pose.pose.orientation.x = 0;
    state.pose.pose.orientation.y = 0;
    state.pose.pose.orientation.z = 0;
    state.pose.pose.orientation.w = 1.0;


    state.twist.twist.linear.x = 0;
    state.twist.twist.linear.y = 0;
    state.twist.twist.linear.z = 0;

    state.twist.twist.angular.x = 0;
    state.twist.twist.angular.y = 0;
    state.twist.twist.angular.z = 0;
  }

  double RandomStateGenerator::NormalDistribution(double mean, double stddev) {
    std::normal_distribution<double> distribution(mean, stddev);
    return distribution(generator);
  }

  double RandomStateGenerator::UniformDistribution(double lower, double upper) {
    std::uniform_real_distribution<double> distribution(lower, upper);
    return distribution(generator);
  }
}

int main (int argc, char** argv) {
  ros::init(argc, argv, "random_state_generator");

  ros::NodeHandle nh;
  rotors_control::RandomStateGenerator random_state_generator(nh);
  ros::spin();

  return 0;
}
