#ifndef _RED_ROBOT_H
#define _RED_ROBOT_H

#include "arf_moveit_wrapper/moveit_wrapper.h"

#include <Eigen/Dense>
#include <string>
#include <vector>

#include "arf_sampling/sampling.h"

class RedundantRobot : public RobotMoveitWrapper
{
    Sampler sampler_;

    public:
    RedundantRobot();
    IKSolution redundantIk(const Eigen::Affine3d& pose, std::vector<double>& q_fixed);
    IKSolution ikGridSamples(const Eigen::Affine3d& pose);
};

#endif