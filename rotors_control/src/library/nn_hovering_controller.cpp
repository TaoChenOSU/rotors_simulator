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

    std::string config_path = model_path;
    config_path.append("nn.config");

    fp.open(config_path.c_str(), std::fstream::in);
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
          ROS_INFO("%d, %d.\n", layer_config.first, layer_config.second);
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

    set_odometry_ = true;
  }

  void NNHoveringController::InitializeNetwork() {
    assert(num_of_layers);

    std::ifstream fp_weights;
    std::ifstream fp_biases;
    std::string line;
    std::vector<std::string> num_in_text;

    std::string weights_path = model_path;
    weights_path.append("weights.txt");
    std::string biases_path = model_path;
    biases_path.append("biases.txt");

    fp_weights.open(weights_path.c_str(), std::fstream::in);
    fp_biases.open(biases_path.c_str(), std::fstream::in);

    if (fp_weights.is_open() && fp_biases.is_open()) {
      for (int i = 0; i < num_of_layers; i++) {
        // read in weights
        num_in_text.clear();
        getline(fp_weights, line);
        boost::split(num_in_text, line, [](char space){return space == ' ';});
        layers_weights.push_back(Eigen::MatrixXf(layers_config[i].first, layers_config[i].second));
        for (int row = 0; row < layers_config[i].first; row++) {
          for (int col = 0; col < layers_config[i].second; col++) {
            layers_weights[i](row, col) = strtof(num_in_text[row*layers_config[i].second+col].c_str(), NULL);
            ROS_INFO("%f ", layers_weights[i](row, col));
          }
        }
        // read in the biases
        num_in_text.clear();
        getline(fp_biases, line);
        boost::split(num_in_text, line, [](char space){return space == ' ';});
        layers_biases.push_back(Eigen::MatrixXf(1, layers_config[i].second));
        for (int row = 0; row < layers_config[i].second; row++) {
          layers_biases[i](0, row) = strtof(num_in_text[row].c_str(), NULL);
          ROS_INFO("%f ", layers_biases[i](0, row));
        }
      }
      fp_weights.close();
      fp_biases.close();
    } else {
      ROS_ERROR("Controller cannot open weights/config files. \n");
      exit(1);
    }
  }

  void NNHoveringController::CalculateRotorVelocities(Eigen::VectorXf* rotor_velocities) const {
    std::vector<Eigen::MatrixXf> layers_result;
    // inputs
    layers_result.push_back(Eigen::MatrixXf(1, layers_config[0].first));
    for (int i = 1; i < num_of_layers; i++) {
      layers_result.push_back(Eigen::MatrixXf(1, layers_config[i].first));
    }
    // outputs
    layers_result.push_back(Eigen::MatrixXf(1, 4));  

    layers_result[0] << position_x, position_y, position_z,
                        orientation_x, orientation_y, orientation_z, orientation_w,
                        linear_vx, linear_vy, linear_vz,
                        angular_vx, angular_vy, angular_vz;

    // evaluate the network
    for (int i = 0; i < num_of_layers-1; i++) {
      layers_result[i+1] = layers_result[i]*layers_weights[i] + layers_biases[i];
      Activation("tanh", &(layers_result[i+1]));
    }
 
    layers_result[num_of_layers] = layers_result[num_of_layers-1]*layers_weights[num_of_layers-1]
                                     + layers_biases[num_of_layers-1];

    rotor_velocities->resize(4);
    (*rotor_velocities)(0) = layers_result[num_of_layers](0, 0)*838;
    (*rotor_velocities)(1) = layers_result[num_of_layers](0, 1)*838;
    (*rotor_velocities)(2) = layers_result[num_of_layers](0, 2)*838;
    (*rotor_velocities)(3) = layers_result[num_of_layers](0, 3)*838;
  }

  void NNHoveringController::Activation(const std::string act, Eigen::MatrixXf* layer) const {
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

  float NNHoveringController::Sigmoid(float n) const{
    return 1/(1+exp(-n));
  }
}
