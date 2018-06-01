/*
** This is a node to read trajectories produced by the neural net controller
** and pass them through the lee controller to get control outputs. (Dagger)
*/

#ifndef DAGGER_ROUTINE_H
#define DAGGER_ROUTINE_H

#include <ros/ros.h>
#include "rotors_control/lee_position_controller.h"

namespace rotors_control {
  class DaggerRoutine {
    public:
      DaggerRoutine();
      ~DaggerRoutine();

    private:
      ros::NodeHandle nh;

      LeePositionController lee_position_controller;

      void LogRotorVelocities();
  }
}
