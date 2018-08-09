/*
** This is a neural network controller that evaluates
** the neural network with a set of weights/biases and outputs the
** control commands to hover.
** This is an experiment.
** Tao Chen, 05/25/2018
*/

#include "rotors_control/nn_hovering_controller.h"

namespace rotors_control{
  NNHoveringController::NNHoveringController() {
    paths_set_ = false;
    trajectory_set_ = false;
  }

  NNHoveringController::~NNHoveringController() {}

  void NNHoveringController::SetModelPath(const char* model, const char* config, const char* stat) {
    weights_path.append(model);
    weights_path.append("/weights.txt");
    biases_path.append(model);
    biases_path.append("/biases.txt");

    config_path.append(config);
    config_path.append("/config.conf");

    stat_path.append(stat);
    stat_path.append("/data_stats.txt");

    paths_set_ = true;
    ROS_INFO("%s\n", config_path.c_str());
    ROS_INFO("%s\n", weights_path.c_str());
    ROS_INFO("%s\n", biases_path.c_str());    
    ROS_INFO("%s\n", stat_path.c_str());    
  }

  void NNHoveringController::init() {
    assert(paths_set_);

    ReadConfig();
    ReadStat();
    InitializeNetwork();
    ROS_INFO("NN controller init completed.\n");
  }

  void NNHoveringController::ReadStat() {
    std::ifstream fp;
    std::string line; 
    std::vector<std::string> line_split;
    std::vector<std::string> nums_in_text;
    std::string type;

    fp.open(stat_path.c_str(), std::fstream::in);
    if (fp.is_open()) {
      while (getline(fp, line)) {
        line_split.clear();
        boost::split(line_split, line, [](char colon){return colon == ':';});
        type = line_split[0];

        boost::split(nums_in_text, line_split[1], [](char space){return space == ' ';});
        if (type.compare(std::string("input_mean")) == 0) {
          for (int i = 0; i < nums_in_text.size(); i++) {
            input_means.push_back(atof(nums_in_text[i].c_str()));
          }
        }
        if (type.compare(std::string("input_std")) == 0) {
          for (int i = 0; i < nums_in_text.size(); i++) {
            input_stds.push_back(atof(nums_in_text[i].c_str()));
          }
        }
        if (type.compare(std::string("output_mean")) == 0) {
          for (int i = 0; i < nums_in_text.size(); i++) {
            output_means.push_back(atof(nums_in_text[i].c_str()));
          }
        }
        if (type.compare(std::string("output_std")) == 0) {
          for (int i = 0; i < nums_in_text.size(); i++) {
            output_stds.push_back(atof(nums_in_text[i].c_str()));
          }
        }
      }
      fp.close();
    } else {
      ROS_ERROR("Controller cannot open stat file. \n");
      exit(1);
    }
  }

  void NNHoveringController::ReadConfig() {
    std::ifstream fp;
    std::string line;
    std::vector<std::string> nums_in_text;
    num_of_layers = 0;

    // regular expression to remove unneccessary characters
    std::regex r("[\\[\\]\\(\\)]+");
    std::string raw_layer_config;

    ROS_INFO("In read config.\n");

    fp.open(config_path.c_str(), std::fstream::in);
    if (fp.is_open()) {
      while (getline(fp, line)) {
        nums_in_text.clear();
        boost::split(nums_in_text, line, [](char colon){return colon == ':';});
        if (nums_in_text[0].compare(std::string("layer_config")) == 0) {
          // use regular expression to parse this line
          raw_layer_config = nums_in_text[1];
          // replace brackets with nothing
          raw_layer_config = std::regex_replace(raw_layer_config, r, "");
          // split the string by comma
          boost::split(nums_in_text, raw_layer_config, [](char comma){return comma == ',';});

          for (int i = 0; i < nums_in_text.size() - 1; i += 2) {
            std::pair<int, int> layer_config;
            layer_config.first = atoi(nums_in_text[i].c_str());
            layer_config.second = atoi(nums_in_text[i+1].c_str());
            layers_config.push_back(layer_config);
            ROS_INFO("%d, %d.\n", layer_config.first, layer_config.second);
          }
        } else if (nums_in_text[0].compare(std::string("act_hidden")) == 0) {
          activations.push_back(std::string(nums_in_text[1].c_str()));
        } else if (nums_in_text[0].compare(std::string("act_output")) == 0) {
          activations.push_back(std::string(nums_in_text[1].c_str()));
        } else {
          ROS_INFO("Unused label");
        }
      }
      num_of_layers = layers_config.size();
      fp.close();
    } else {
      ROS_ERROR("Controller cannot open config file. \n");
      exit(1);
    }
  }

  void NNHoveringController::InitializeNetwork() {
    assert(num_of_layers);

    std::ifstream fp_weights;
    std::ifstream fp_biases;
    std::string line;
    std::vector<std::string> num_in_text;

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
          }
        }
        // read in the biases
        num_in_text.clear();
        getline(fp_biases, line);
        boost::split(num_in_text, line, [](char space){return space == ' ';});
        layers_biases.push_back(Eigen::MatrixXf(1, layers_config[i].second));
        for (int row = 0; row < layers_config[i].second; row++) {
          layers_biases[i](0, row) = strtof(num_in_text[row].c_str(), NULL);
        }
      }
      fp_weights.close();
      fp_biases.close();
    } else {
      ROS_ERROR("Controller cannot open weights/biases files. \n");
      exit(1);
    }
  }

  void NNHoveringController::SetOdometry(const nav_msgs::OdometryConstPtr& odometry_msg) {
    // calculate the error between the current state and the trajectories set point
    // error is computed according to the lee controller

    EigenOdometry odometry;
    eigenOdometryFromMsg(odometry_msg, &odometry);
    
    // position error 
    Eigen::Vector3d position_error;
    position_error = odometry.position - command_trajectory_.position_W;
    position_x_e = position_error[0];
    position_y_e = position_error[1];
    position_z_e = position_error[2];

    ROS_INFO("CONTROLLER_INPUT_STATES: %f %f %f", odometry.position[0], odometry.position[1], odometry.position[2]);
    ROS_INFO("CONTROLLER_ERROR: %f %f %f", position_x_e, position_y_e, position_z_e);



    // orientation
    rotational_matrix = odometry.orientation.toRotationMatrix();

    // linear velocity error
    // Transform velocity to world frame 
    Eigen::Vector3d velocity_W = rotational_matrix * odometry.velocity;

    linear_vx_e = velocity_W[0];
    linear_vy_e = velocity_W[1];
    linear_vz_e = velocity_W[2];    

    // assuming angular velocity is always (0, 0, 0)
    angular_vx = odometry_msg->twist.twist.angular.x;
    angular_vy = odometry_msg->twist.twist.angular.y;
    angular_vz = odometry_msg->twist.twist.angular.z;

    /*
    * If using the rotational matrix to represent the difference between the current orientation
    * and desired orientation, uncomment the statement below.
    */
    // rotational_matrix = rotational_matrix*R_des_f.transpose();
  }

  void NNHoveringController::SetTrajectoryPoint(
      const mav_msgs::EigenTrajectoryPoint& command_trajectory) {
    command_trajectory_ = command_trajectory;

    yaw = command_trajectory_.getYaw();
    cos_yaw = (float)cos(yaw);
    sin_yaw = (float)sin(yaw);

    R_des_f << cos_yaw, -sin_yaw, 0, 
               sin_yaw, cos_yaw, 0,
               0, 0, 1;

    trajectory_set_ = true;
  }
 
  void NNHoveringController::CalculateRotorVelocities(Eigen::VectorXf* rotor_velocities) const {
    rotor_velocities->resize(4);
    if(!trajectory_set_) {
      *rotor_velocities = Eigen::VectorXf::Zero(rotor_velocities->rows());
      return;
    }
    std::vector<Eigen::MatrixXf> layers_result;

    // activations
    if (activations.size() != 2) {
      ROS_ERROR("Activations not set...\n");
      exit(1);
    }
    std::string hidden_activation = activations[0];
    std::string output_activation = activations[1];

    // inputs
    layers_result.push_back(Eigen::MatrixXf(1, layers_config[0].first));
    for (int i = 1; i < num_of_layers; i++) {
      layers_result.push_back(Eigen::MatrixXf(1, layers_config[i].first));
    }
    // outputs
    layers_result.push_back(Eigen::MatrixXf(1, 4));  

    layers_result[0] << position_x_e, position_y_e, position_z_e,
                        linear_vx_e, linear_vy_e, linear_vz_e,
                        rotational_matrix(0, 0), rotational_matrix(0, 1), rotational_matrix(0, 2),
                        rotational_matrix(1, 0), rotational_matrix(1, 1), rotational_matrix(1, 2),
                        rotational_matrix(2, 0), rotational_matrix(2, 1), rotational_matrix(2, 2),  
                        angular_vx, angular_vy, angular_vz,
                        cos_yaw, sin_yaw;   // comment this out if using rotational matrix to represent the difference

    // standardize the inputs
    for (int i = 0; i < input_means.size(); i++) {
      layers_result[0](0, i) = (layers_result[0](0, i) - input_means[i]) / input_stds[i];
    }
                                         
    // evaluate the network
    for (int i = 0; i < num_of_layers-1; i++) {
      layers_result[i+1] = layers_result[i]*layers_weights[i] + layers_biases[i];
      Activation(hidden_activation, &(layers_result[i+1]));
    }
 
    layers_result[num_of_layers] = layers_result[num_of_layers-1]*layers_weights[num_of_layers-1]
                                     + layers_biases[num_of_layers-1];

    Activation(output_activation, &(layers_result[num_of_layers]));

    rotor_velocities->resize(4);
    (*rotor_velocities)(0) = layers_result[num_of_layers](0, 0) * output_stds[0] + output_means[0];
    (*rotor_velocities)(1) = layers_result[num_of_layers](0, 1) * output_stds[1] + output_means[1];
    (*rotor_velocities)(2) = layers_result[num_of_layers](0, 2) * output_stds[2] + output_means[2];
    (*rotor_velocities)(3) = layers_result[num_of_layers](0, 3) * output_stds[3] + output_means[3];
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
    } else if (act.compare("relu") == 0) {
      for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
          (*layer)(i, j) = Relu((*layer)(i, j));
        }
      }
    } else if (act.compare("linear") == 0) {
      // linear doesn't need any operations
      return;
    } else {
      ROS_ERROR("No activation function.\n");
      exit(1);
    }
  }

  float NNHoveringController::Sigmoid(float n) const{
    return 1/(1+exp(-n));
  }

  float NNHoveringController::Relu(float n) const {
    if (n < 0) {
      return 0;
    } else {
      return n;
    }
  }
}
