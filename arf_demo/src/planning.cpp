#include <ros/ros.h>

// #include <chrono>
#include <memory>
#include <iostream>
#include <map>
#include <functional>

#include <simple_moveit_wrapper/industrial_robot.h>
#include <arf_tsr/task_space_region.h>
#include <arf_sampling/random_sampler.h>
// #include <arf_sampling/grid_sampler.h>
// #include <arf_sampling/halton_sampler.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

namespace rvt = rviz_visual_tools;
namespace smw = simple_moveit_wrapper;

class Rviz
{
public:
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
  Rviz()
  {
    visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("base_link", "/visualization_marker_array"));
    visual_tools_->loadMarkerPub(true);
    visual_tools_->loadRobotStatePub("/display_robot_state", true);
  }

  void plotPose(Eigen::Isometry3d pose);
  void clear();
};

std::vector<std::vector<double>> ikSampling(smw::Robot& robot, const arf::TSR& tsr,
                                            const std::size_t min_samples);

static constexpr double INF{ std::numeric_limits<double>::infinity() };

// typedef std::shared_ptr<std::vector<double>> VectorPtr;

struct Node
{
  std::vector<double>* data = nullptr;
  double shortest_distance = INF;
  Node* parent = nullptr;
  bool visited = false;
  std::size_t id{ 0 };

  static std::size_t num_nodes;
  Node()
  {
    id = num_nodes;
    num_nodes++;
  }
};

std::size_t Node::num_nodes = 0;

struct Graph
{
  std::vector<std::vector<double>> data;
  std::vector<Node> nodes;
  std::map<std::size_t, std::vector<std::size_t>> edges;
  // std::vector<std::vector<Node&>> edges;
};

std::ostream& operator<<(std::ostream& os, const std::vector<double>& v)
{
  os << "( ";
  for (auto& value : v)
    os << value << ", ";
  os << ")";
  return os;
}

std::ostream& operator<<(std::ostream& os, const Node& node)
{
  os << "Node: " << *(node.data);
  os << " dist: " << node.shortest_distance;
  os << " vis: " << (node.visited ? "true" : "false");
  return os;
}

std::ostream& operator<<(std::ostream& os, const Graph& graph)
{
  os << "Graph: \n";
  os << "Nodes:\n";
  for (auto& n : graph.nodes)
    os << n << "\n";
  os << "Edges:\n";
  for (auto& e : graph.edges)
  {
    os << graph.nodes[e.first].id << "\n";
    for (auto& to : e.second)
      os << "\t-> " << graph.nodes[to].id << "\n";
  }
  return os;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "demo_planning");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // double BOX_SIZE{ 0.2 };

  simple_moveit_wrapper::IndustrialRobot robot;
  Rviz rviz;
  rviz.clear();

  std::vector<double> home = { 0, -1.5, 1.5, 0, 0, 0 };
  robot.plot(rviz.visual_tools_, home);

  arf::Transform tf_nominal(robot.fk(home));
  rviz.plotPose(tf_nominal);

  double sbox{ 0.5 };
  arf::TSRBounds bounds_2{ { { -sbox / 2, sbox / 2 },
                             { -sbox / 2, sbox / 2 },
                             { -sbox / 2, sbox / 2 },
                             { -M_PI, M_PI },
                             { -M_PI, M_PI },
                             { -M_PI, M_PI } } };
  std::vector<int> ns_2{ 3, 3, 3, 2, 2, 2 };

  arf::TSR tsr_2(tf_nominal, bounds_2, std::make_shared<arf::RandomSampler>(), ns_2);
  // arf::TSR tsr_2(tf_nominal, bounds_2, std::make_shared<arf::GridSampler>(), ns_2);
  // arf::TSR tsr_2(tf_nominal, bounds_2, std::make_shared<arf::HaltonSampler>(), ns_2);

  auto samples1 = ikSampling(robot, tsr_2, 5);
  ROS_INFO_STREAM("Found " << samples1.size() << " samples");

  Graph graph;

  for (auto& jp : samples1)
  {
    robot.plot(rviz.visual_tools_, jp);
    ros::Duration(0.05).sleep();
    rviz.plotPose(robot.fk(jp));

    // std::cout << jp << std::endl;

    Node n{};
    n.data = &jp;
    graph.data.push_back(jp);
    graph.nodes.push_back(n);
  }

  graph.edges[0] = { 1, 3 };
  graph.edges[3] = { 4, 5, 6 };

  std::cout << graph << std::endl;

  ros::shutdown();

  return 0;
}

void Rviz::plotPose(Eigen::Isometry3d pose)
{
  visual_tools_->publishAxis(pose, rvt::LARGE);
  visual_tools_->trigger();
}

void Rviz::clear()
{
  visual_tools_->deleteAllMarkers();
  visual_tools_->trigger();
}

std::vector<std::vector<double>> ikSampling(smw::Robot& robot, const arf::TSR& tsr,
                                            const std::size_t min_samples)
{
  const std::size_t max_iters{ 2000 };

  std::vector<std::vector<double>> samples;
  samples.reserve(min_samples + 10);  // could be more because of IK solutions

  std::size_t num_samples{ 0 };
  for (std::size_t iter{ 0 }; iter < max_iters; ++iter)
  {
    auto tsamples = tsr.getSamples();
    for (auto& pose : tsamples)
    {
      auto iksols = robot.ik(pose);
      for (auto& js : iksols)
      {
        samples.push_back(js);
      }
      num_samples += iksols.size();
      if (num_samples >= min_samples)
      {
        break;
      }
    }

    if (num_samples >= min_samples)
    {
      // ROS_INFO_STREAM("IK sampler found " << num_samples << " in " << iter << " iterations.");
      break;
    }
  }
  samples.shrink_to_fit();
  return samples;
}
