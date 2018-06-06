/*
** This is a node to read trajectories produced by the neural net controller
** and pass them through the lee controller to get control outputs. (Dagger)
*/

#include "dagger_routine.h"

namespace rotors_control {

	DaggerRoutine::DaggerRoutine(const ros::NodeHandle& nh) : nh_(nh) {
		assert(ReadStates());

		state_pub_ = nh_.advertise<nav_msgs::Odometry>(
			mav_msgs::default_topics::ODOMETRY, 1); 

		trajectory_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
			mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);

		WaypointPub();

		command_timer_ = nh_.createTimer(ros::Duration(0.1), &DaggerRoutine::PublishState, this);
	} 
  
  	DaggerRoutine::~DaggerRoutine() {}

  	int DaggerRoutine::ReadStates() {
  		std::ifstream fp;
		std::string line;
		std::vector<std::string> nums_in_text;

		int num_of_states = 0;

		fp.open("/home/taotaochen/Desktop/REL_Lab/RotorS_Logger/policy_states.csv");
		if (fp.is_open()) {
			while (getline(fp, line)) {
				nums_in_text.clear();
				boost::split(nums_in_text, line, [](char comma){return comma == ',';});
				if (nums_in_text.size() != 13) {
					ROS_ERROR("Reading policy state error.\n");
					exit(1);
				} else {
					nav_msgs::Odometry odometry_msg; 
					odometry_msg.pose.pose.position.x = atof(nums_in_text[0].c_str());
					odometry_msg.pose.pose.position.y = atof(nums_in_text[1].c_str());
					odometry_msg.pose.pose.position.z = atof(nums_in_text[2].c_str());
					odometry_msg.pose.pose.orientation.x = atof(nums_in_text[3].c_str());
					odometry_msg.pose.pose.orientation.y = atof(nums_in_text[4].c_str());
					odometry_msg.pose.pose.orientation.z = atof(nums_in_text[5].c_str());
					odometry_msg.pose.pose.orientation.w = atof(nums_in_text[6].c_str());
					odometry_msg.twist.twist.linear.x = atof(nums_in_text[7].c_str());
					odometry_msg.twist.twist.linear.y = atof(nums_in_text[8].c_str());
					odometry_msg.twist.twist.linear.z = atof(nums_in_text[9].c_str());
					odometry_msg.twist.twist.angular.x = atof(nums_in_text[10].c_str());
					odometry_msg.twist.twist.angular.y = atof(nums_in_text[11].c_str());
					odometry_msg.twist.twist.angular.z = atof(nums_in_text[12].c_str());

					odometry_msgs.push_back(odometry_msg);
				}
			}
			num_of_states = odometry_msgs.size();
			fp.close();

			return num_of_states;
		} else {
			ROS_ERROR("Cannot open policy state file.\n");
			exit(0);
		}
  	}

  	void DaggerRoutine::WaypointPub() {
	    trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
	    trajectory_msg.header.stamp = ros::Time::now();

	    Eigen::Vector3d desired_position(0.0, 0.0, 1.0);
	    double desired_yaw = 0.0;

	    mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
	      desired_position, desired_yaw, &trajectory_msg);

	    ros::Duration(5.0).sleep();

	    trajectory_pub_.publish(trajectory_msg);
	    ROS_INFO("Way point published: [%f, %f, %f]",
	            desired_position.x(), desired_position.y(), desired_position.z());
	}

	void DaggerRoutine::PublishState(const ros::TimerEvent& e) {
		if (odometry_msgs.size() == 0) {
			ROS_INFO("Finished.\n");
			exit(0);
		}

	    nav_msgs::Odometry state;
	    state = odometry_msgs.back();
	    odometry_msgs.pop_back();
	    state_pub_.publish(state);
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "Dagger Routine");

	ros::NodeHandle nh;
	rotors_control::DaggerRoutine dagger_routine(nh);
	ros::spin();

	return 0;
}
