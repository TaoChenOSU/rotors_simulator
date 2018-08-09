/*
** This is a neural network controller that evaluates
** the neural network with a set of weights and outputs the
** control commands to hover.
** This is an experiment.
** Tao Chen, 05/25/2018
*/

#ifndef ROTORS_CONTROL_NN_HOVERING_CONTROLLER_H
#define ROTORS_CONTROL_NN_HOVERING_CONTROLLER_H

#include <boost/algorithm/string.hpp>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <ros/ros.h>

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>

#include "rotors_control/common.h"

#include <fstream>
#include <math.h>
#include <string.h>
#include <regex>
#include <iterator>

#define MAX_RPM 838

namespace rotors_control {
  class NNHoveringController{
    public:
      NNHoveringController();
      ~NNHoveringController();

      void SetTrajectoryPoint(const mav_msgs::EigenTrajectoryPoint& command_trajectory);

      void SetOdometry(const nav_msgs::OdometryConstPtr& odometry_msg);

      void CalculateRotorVelocities(Eigen::VectorXf* rotor_velocities) const;

      void SetModelPath(const char* model, const char* config, const char* stat);

      void init();

    private:
      float position_x_e, position_y_e, position_z_e;
      Eigen::Matrix3d rotational_matrix, R_des_f, R_diff;
      float linear_vx_e, linear_vy_e, linear_vz_e;
      float angular_vx, angular_vy, angular_vz;
      double yaw;
      float cos_yaw, sin_yaw;

      bool trajectory_set_;

      mav_msgs::EigenTrajectoryPoint command_trajectory_;

      int num_of_layers;
      std::vector<std::pair<int, int> > layers_config;
      std::vector<Eigen::MatrixXf> layers_weights;
      std::vector<Eigen::MatrixXf> layers_biases;
      std::vector<std::string> activations;

      std::vector<float> input_means;
      std::vector<float> input_stds;
      std::vector<float> output_means;
      std::vector<float> output_stds;

      // file path to find the config file and weights/biases files
      std::string weights_path;
      std::string biases_path;
      std::string config_path;
      std::string stat_path;
      bool paths_set_;

      void ReadConfig();

      void ReadStat();

      void InitializeNetwork();

      // void NormalizeInput();

      void Activation(const std::string act, Eigen::MatrixXf* layer) const;

      float Sigmoid(float n) const;

      float Relu(float n) const;
  };

}

#endif
