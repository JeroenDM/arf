#include "arf_trajectory/trajectory.h"

void TolerancedNumber::checkInput()
{
    if (lower_bound_ >= upper_bound_)
    {
        throw std::invalid_argument("Lower bound should be strictly smaller than upper bound.");
    }
    if (nominal_ < lower_bound_ || nominal_ > upper_bound_)
    {
        throw std::invalid_argument("Nominal value should be withing bounds.");
    }
}

TrajectoryPoint::TrajectoryPoint(Number& x, Number& y, Number& z, Number& rx, Number& ry, Number& rz, double timing)
{
    raw_numbers_[0] = &x;
    raw_numbers_[1] = &y;
    raw_numbers_[2] = &z;
    raw_numbers_[3] = &rx;
    raw_numbers_[4] = &ry;
    raw_numbers_[5] = &rz;
    std::vector<double> nominal_values = { x, y, z, rx, ry, rz };
    nominal_pose_ = valuesToPose(nominal_values);

    for (auto n : raw_numbers_)
    {
        sampler_.addDimension(n->lower_bound_, n->upper_bound_, n->num_samples_);
    }
    time_from_previous_point_ = timing;
}

Transform TrajectoryPoint::valuesToPose(std::vector<double>& values)
{
    using Translation = Eigen::Translation3d;
    using AngleAxis = Eigen::AngleAxisd;
    using Vector = Eigen::Vector3d;

    // clang-format off
    Transform t;
      t = Translation(values[0], values[1], values[2]) *
          AngleAxis(values[3], Vector::UnitX()) *
          AngleAxis(values[4], Vector::UnitY()) *
          AngleAxis(values[5], Vector::UnitZ());
      return t;
    // clang-format on
}

std::vector<Transform> TrajectoryPoint::getGridSamples()
{
    std::vector<Transform> poses;

    for (auto val : sampler_.getGridSamples())
    {
        poses.push_back(valuesToPose(val));
    }
    return poses;
}

void TrajectoryPoint::plot(moveit_visual_tools::MoveItVisualToolsPtr mvt)
{
    mvt->publishAxis(nominal_pose_, rviz_visual_tools::LARGE);
    mvt->trigger();
}