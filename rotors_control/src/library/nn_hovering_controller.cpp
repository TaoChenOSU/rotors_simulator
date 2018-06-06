/*
** This is a neural network controller that evaluates
** the neural network with a set of weights and outputs the
** control commands to hover.
** This is an experiment.
** Tao Chen, 05/25/2018
*/

#include "rotors_control/nn_hovering_controller.h"

namespace rotors_control{
  NNHoveringController::NNHoveringController() {
    std::ifstream fp;
    std::string line;
    std::vector<std::string> nums_in_text;
    num_of_layers = 0;

    fp.open("/home/taotaochen/Desktop/REL_Lab/NN_train/nn.config", std::fstream::in);
    if (fp.is_open()) {
      while (getline(fp, line)) {
        nums_in_text.clear();
        std::pair<int, int> layer_config;
        boost::split(nums_in_text, line, [](char space){return space == ' ';});
        if (nums_in_text.size() != 2) {
          ROS_ERROR("Controller reading config file error. \n");
          exit(1);
        } else {
          layer_config.first = atoi(nums_in_text[0].c_str());
          layer_config.second = atoi(nums_in_text[1].c_str());
          layers_config.push_back(layer_config);
          // ROS_INFO("%d, %d.\n", layer_config.first, layer_config.second);
        }
      }
      num_of_layers = layers_config.size();
      InitializeNetwork();
      fp.close();
    } else {
      ROS_ERROR("Controller cannot open config file. \n");
      exit(1);
    }
  }

  NNHoveringController::~NNHoveringController() {}

  void NNHoveringController::SetOdometry(const nav_msgs::OdometryConstPtr& odometry_msg) {
    position_x = odometry_msg->pose.pose.position.x;
    position_y = odometry_msg->pose.pose.position.y;
    position_z = odometry_msg->pose.pose.position.z;

    orientation_x = odometry_msg->pose.pose.orientation.x;
    orientation_y = odometry_msg->pose.pose.orientation.y;
    orientation_z = odometry_msg->pose.pose.orientation.z;
    orientation_w = odometry_msg->pose.pose.orientation.w;

    linear_vx = odometry_msg->twist.twist.linear.x;
    linear_vy = odometry_msg->twist.twist.linear.y;
    linear_vz = odometry_msg->twist.twist.linear.z;

    angular_vx = odometry_msg->twist.twist.angular.x;
    angular_vy = odometry_msg->twist.twist.angular.y;
    angular_vz = odometry_msg->twist.twist.angular.z;

    // NormalizeInput();

    set_odometry_ = true;
  }

  void NNHoveringController::InitializeNetwork() {
    assert(num_of_layers);

    std::ifstream fp_weights;
    std::ifstream fp_biases;
    std::string line;
    std::vector<std::string> num_in_text;

    fp_weights.open("/home/taotaochen/Desktop/REL_Lab/NN_train/weights.txt", std::fstream::in);
    fp_biases.open("/home/taotaochen/Desktop/REL_Lab/NN_train/biases.txt", std::fstream::in);

    if (fp_weights.is_open() && fp_biases.is_open()) {
      for (int i = 0; i < num_of_layers; i++) {
        // read in weights
        num_in_text.clear();
        getline(fp_weights, line);
        boost::split(num_in_text, line, [](char space){return space == ' ';});
        layers_weights.push_back(Eigen::MatrixXd(layers_config[i].first, layers_config[i].second));
        for (int row = 0; row < layers_config[i].first; row++) {
          for (int col = 0; col < layers_config[i].second; col++) {
            layers_weights[i](row, col) = atof(num_in_text[row*layers_config[i].second+col].c_str());
            // ROS_INFO("%f ", layers_weights[i](row, col));
          }
        }
        // read in the biases
        num_in_text.clear();
        getline(fp_biases, line);
        boost::split(num_in_text, line, [](char space){return space == ' ';});
        layers_biases.push_back(Eigen::MatrixXd(1, layers_config[i].second));
        for (int row = 0; row < layers_config[i].second; row++) {
          layers_biases[i](0, row) = atof(num_in_text[row].c_str());
          // ROS_INFO("%f ", layers_biases[i](0, row));
        }
      }
      fp_weights.close();
      fp_biases.close();
    }
  }

  void NNHoveringController::NormalizeInput() {
    return;
  }

  void NNHoveringController::CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities) const {
    Eigen::MatrixXd state(1, 13);
    Eigen::MatrixXd inter_result_1(1, 64);
    Eigen::MatrixXd inter_result_2(1, 64);    
    Eigen::MatrixXd result(1, 4);
    state << position_x, position_y, position_z,
                    orientation_x, orientation_y, orientation_z, orientation_w,
                    linear_vx, linear_vy, linear_vz,
                    angular_vx, angular_vy, angular_vz;

    inter_result_1 = state*layers_weights[0] + layers_biases[0];
    Activation("tanh", &inter_result_1);
    inter_result_2 = inter_result_1*layers_weights[1] + layers_biases[1];
    Activation("tanh", &inter_result_2); 
    result = inter_result_2*layers_weights[2] + layers_biases[2];

    rotor_velocities->resize(4);
    (*rotor_velocities)(0) = result(0, 0)*838;
    (*rotor_velocities)(1) = result(0, 1)*838;
    (*rotor_velocities)(2) = result(0, 2)*838;
    (*rotor_velocities)(3) = result(0, 3)*838;
  }

  void NNHoveringController::Activation(const std::string act, Eigen::MatrixXd* layer) const {
    int rows = layer->rows();
    int cols = layer->cols();

    // tanh activation
    if (act.compare("tanh") == 0) {
      for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
          (*layer)(i, j) = tanh((*layer)(i, j));
        }
      }
    } else if (act.compare("sigmoid") == 0) {
      for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
          (*layer)(i, j) = Sigmoid((*layer)(i, j));
        }
      }
    } else {
      ROS_ERROR("No activation function.\n");
    }
  }

  double NNHoveringController::Sigmoid(double n) const{
    return 1/(1+exp(-n));
  }
}
