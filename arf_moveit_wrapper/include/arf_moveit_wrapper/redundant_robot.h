#include "arf_moveit_wrapper/moveit_wrapper.h"

#include <Eigen/Dense>
#include <string>
#include <vector>

class RedundantRobot : public RobotMoveitWrapper
{
    public:
    IKSolution redundantIk(const Eigen::Affine3d pose, std::vector<double>& q_fixed);
};