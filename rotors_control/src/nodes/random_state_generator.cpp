/*
** This is a node to generate state randomly based on certain
** distributions to speed up the data collection process on controllers.
*/

#include "random_state_generator.h"

namespace rotors_control {
  RandomStateGenerator::RandomStateGenerator(const ros::NodeHandle& nh)
    : nh_(nh) {

    // state_pub_ = nh_.advertise<nav_msgs::Odometry>(
    //   mav_msgs::default_topics::ODOMETRY, 1);

    set_state_srv = nh_.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

    trajectory_pub = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);

    // WaypointPub();

    command_timer_ = nh_.createTimer(ros::Duration(2.0), &RandomStateGenerator::SetState, this);

    counter = 0;
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

  // void RandomStateGenerator::PublishState(const ros::TimerEvent& e) {
  //   if(counter%1000 == 0){
  //     ROS_INFO("%d points.\n", counter);
  //   }
  //   counter++;
  //   nav_msgs::Odometry state;
  //   GetNewState(state);
  //   state_pub_.publish(state);
  // }

  void RandomStateGenerator::SetState(const ros::TimerEvent& e) {
    nav_msgs::Odometry state;
    GetNewState(state);
    gazebo_msgs::SetModelState srv;

    srv.request.model_state.model_name = "hummingbird";
    srv.request.model_state.reference_frame = "world";
    srv.request.model_state.pose.position.x = state.pose.pose.position.x;
    srv.request.model_state.pose.position.y = state.pose.pose.position.y;
    srv.request.model_state.pose.position.z = state.pose.pose.position.z;

    srv.request.model_state.pose.orientation.x = state.pose.pose.orientation.x;
    srv.request.model_state.pose.orientation.y = state.pose.pose.orientation.y;
    srv.request.model_state.pose.orientation.z = state.pose.pose.orientation.z;
    srv.request.model_state.pose.orientation.w = state.pose.pose.orientation.w;

    srv.request.model_state.twist.linear.x = state.twist.twist.linear.x;
    srv.request.model_state.twist.linear.y = state.twist.twist.linear.y;
    srv.request.model_state.twist.linear.z = state.twist.twist.linear.z;

    srv.request.model_state.twist.angular.x = state.twist.twist.angular.x;
    srv.request.model_state.twist.angular.y = state.twist.twist.angular.y;
    srv.request.model_state.twist.angular.z = state.twist.twist.angular.z;

    if (!set_state_srv.call(srv)) {
      ROS_ERROR("Failed to call service to set the state.\n");
      exit(1);
    }
    if(counter%10 == 0){
      ROS_INFO("%d points.\n", counter);
    }
    counter++;
  }

  // only positions vary.
  // velocities and orientations are collected through simulation (Dagger)
  void RandomStateGenerator::GetNewState(nav_msgs::Odometry& state) {
    // double normalizedFactor;
    double roll, pitch, yaw;

    state.pose.pose.position.x = NormalDistribution(0.0, 0.1);
    state.pose.pose.position.y = NormalDistribution(0.0, 0.1);
    state.pose.pose.position.z = UniformDistribution(0.0, 1.2);

    roll = NormalDistribution(0.0, PI/10);
    pitch = NormalDistribution(0.0, PI/10);
    yaw = NormalDistribution(0.0, PI/10);

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

    // state.pose.pose.orientation.x = 0;
    // state.pose.pose.orientation.y = 0;
    // state.pose.pose.orientation.z = 0;
    // state.pose.pose.orientation.w = 1.0;

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
