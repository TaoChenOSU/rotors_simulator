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

#include <fstream>
#include <math.h>
#include <string.h>

namespace rotors_control {
  class NNHoveringController{
    public:
      NNHoveringController();
      ~NNHoveringController();

      void SetOdometry(const nav_msgs::OdometryConstPtr& odometry_msg);

      void CalculateRotorVelocities(Eigen::VectorXf* rotor_velocities) const;

    private:
      float position_x, position_y, position_z;
      float orientation_x, orientation_y, orientation_z, orientation_w;
      float linear_vx, linear_vy, linear_vz;
      float angular_vx, angular_vy, angular_vz;

      bool set_odometry_;

      int num_of_layers;
      std::vector<std::pair<int, int> > layers_config;
      std::vector<Eigen::MatrixXf> layers_weights;
      std::vector<Eigen::MatrixXf> layers_biases;

      std::string model_path = "/home/taotaochen/Desktop/REL_Lab/NN_train/models/8/";

      void InitializeNetwork();

      // void NormalizeInput();

      void Activation(const std::string act, Eigen::MatrixXf* layer) const;

      float Sigmoid(float n) const;
  };

}

#endif
