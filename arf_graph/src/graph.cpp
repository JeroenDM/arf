#include "arf_graph/graph.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <queue>

double L1NormCost(Node n1, Node n2)
{
  double cost = 0;
  int s = (*n1.jv).size();
  for (int i = 0; i < (*n1.jv).size(); ++i)
  {
    cost += std::fabs((*n1.jv)[i] - (*n2.jv)[i]);
  }
  return cost;
}

double sumSquaredCost(Node n1, Node n2)
{
  double cost = 0;
  int s = (*n1.jv).size();
  for (int i = 0; i < (*n1.jv).size(); ++i)
  {
    cost += ((*n1.jv)[i] - (*n2.jv)[i]) * ((*n1.jv)[i] - (*n2.jv)[i]);
  }
  return cost;
}

Graph::Graph(std::vector<std::vector<JointPose>>& path_joint_poses)
{
  raw_data_ = path_joint_poses;
  num_path_points_ = path_joint_poses.size();
  for (int i = 0; i < num_path_points_; ++i)
  {
    std::vector<Node> new_nodes;
    for (int j = 0; j < path_joint_poses[i].size(); ++j)
    {
      Node n{};
      n.path_index = i;
      n.sample_index = j;
      n.jv = &path_joint_poses[i][j];
      new_nodes.push_back(n);
    }
    nodes_.push_back(new_nodes);
  }
}

void Graph::addPathPointData(std::vector<JointPose>& joint_poses)
{
  raw_data_.push_back(joint_poses);
  std::vector<Node> new_nodes;
  for (int i = 0; i < joint_poses.size(); ++i)
  {
    Node n{};
    n.path_index = num_path_points_;
    n.sample_index = i;
    n.jv = &joint_poses[i];
    new_nodes.push_back(n);
  }
  nodes_.push_back(new_nodes);
  num_path_points_++;
}

const std::vector<std::vector<Node>>& Graph::getNodes() const
{
  return nodes_;
}

std::vector<Node*> Graph::getNeighbors(Node* node)
{
  std::vector<Node*> neighbors;
  if ((*node).path_index != (num_path_points_ - 1))
  {
    for (Node& n : nodes_[(*node).path_index + 1])
    {
      neighbors.push_back(&n);
    }
  }
  return neighbors;
}

void Graph::runMultiSourceDijkstra()
{
  // std::queue<Node*> Q;
  std::priority_queue<Node*, std::vector<Node*>, sortNodesFunction> Q;
  Node* current;
  std::vector<Node*> neighbors;

  static Node DUMMY_NODE{};

  // one loop does two things
  // 1. add dummy parent node before all start nodes
  // 2. add all start nodes to the queue
  for (Node& sn : nodes_[0])
  {
    sn.shortest_distance = 0;
    sn.parent = &DUMMY_NODE;
    sn.visited = true;
    Q.push(&sn);
  }

  int num_goals_to_visit = nodes_[num_path_points_ - 1].size();

  while (!Q.empty() and (num_goals_to_visit > 0))
  {
    // get the next node in line
    // current = Q.front();
    current = Q.top();
    Q.pop();

    // count the number of goal nodes that are reached
    if ((*current).path_index == (num_path_points_ - 1))
    {
      num_goals_to_visit--;
    }

    neighbors = getNeighbors(current);
    for (Node* nb : neighbors)
    {
      // update neighbors distance
      float dist = (*current).shortest_distance + L1NormCost(*nb, *current);
      if (dist < (*nb).shortest_distance)
      {
        (*nb).shortest_distance = dist;
        (*nb).parent = current;
      }
      // add to queue if not yet visited
      if (!(*nb).visited)
      {
        Q.push(nb);
        (*nb).visited = true;
      }
    }
  }

  shortest_path_is_found = true;
}

std::vector<Node*> Graph::getShortestPath()
{
  std::vector<Node*> path;

  if (shortest_path_is_found)
  {
    // find last node with shortest distance to start
    float min_dist = INF;
    Node* goal;
    for (auto& n : nodes_.back())
    {
      if (n.shortest_distance < min_dist)
      {
        goal = &n;
        min_dist = n.shortest_distance;
      }
    }

    last_path_cost = min_dist;

    Node* current_node = goal;
    while ((*current_node).path_index > 0)
    {
      path.push_back(current_node);
      current_node = (*current_node).parent;
    }
    path.push_back(current_node);
    std::reverse(path.begin(), path.end());
  }
  else
  {
    std::cout << "No path found" << std::endl;
  }
  return path;
}