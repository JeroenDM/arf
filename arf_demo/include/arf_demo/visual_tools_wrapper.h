#ifndef _VISUAL_TOOLS_WRAPPER_H_
#define _VISUAL_TOOLS_WRAPPER_H_

#include <Eigen/Dense>
#include <moveit_visual_tools/moveit_visual_tools.h>

namespace arf
{
namespace rvt = rviz_visual_tools;

class Rviz
{
public:
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
  Rviz()
  {
    visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("world", "/rviz_visual_tools"));
  }

  void plotPose(Eigen::Affine3d pose);
  void plotPath(Robot& robot, std::vector<std::vector<double>>& path);
  void clear();
};

void Rviz::plotPose(Eigen::Affine3d pose)
{
  Eigen::Isometry3d pose_temp(pose.matrix());
  visual_tools_->publishAxis(pose_temp, rvt::LARGE);
  visual_tools_->trigger();
}

void Rviz::plotPath(Robot& robot, std::vector<std::vector<double>>& path)
{
  for (auto q : path)
  {
    robot.plot(visual_tools_, q);
    ros::Duration(0.5).sleep();
  }

}
void Rviz::clear()
{
  visual_tools_->deleteAllMarkers();
  visual_tools_->trigger();
  // short pause to make sure everything is deleted
  ros::Duration(0.1).sleep();
}

} // namespace arf

#endif // _VISUAL_TOOLS_WRAPPER_H_