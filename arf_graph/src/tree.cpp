#include "arf_graph/tree.h"

#include <queue>
#include <algorithm>  // std::find

namespace arf
{
void Node::reset()
{
    dist = std::numeric_limits<double>::max();
    parent = nullptr;
    visited = false;
}

struct compareNodesFunction
{
    bool operator()(NodePtr n1, NodePtr n2) const
    {
        return (*n2).dist < (*n1).dist;
    }
};

std::vector<NodePtr> _extract_path(const Tree& tree, NodePtr goal)
{
    std::vector<NodePtr> path;
    path.push_back(goal);
    NodePtr node = goal;
    while (true)
    {
        if (node->parent == nullptr)
        {
            break;
        }
        else
        {
            node = node->parent;
            path.insert(path.begin(), node);
        }
    }

    return path;
}

std::vector<NodePtr> shortest_path(Tree& tree, NodePtr start, NodePtr goal)
{
    //  std::queue<Node*> Q;
    std::priority_queue<NodePtr, std::vector<NodePtr>, compareNodesFunction> Q;

    start->dist = start->cost;

    Q.push(start);

    NodePtr current_node{ nullptr };
    std::vector<Edge> nb;

    while (!Q.empty())
    {
        //    current_node = Q.front();
        current_node = Q.top();
        Q.pop();

        if (current_node == goal)
        {
            break;
        }

        if (tree.find(current_node) == tree.end())
        {
            continue;
        }
        else
        {
            nb = tree[current_node];
        }

        for (auto edge : nb)
        {
            if (current_node->dist + edge.cost + edge.child->cost < edge.child->dist)
            {
                edge.child->dist = current_node->dist + edge.cost + edge.child->cost;
                edge.child->parent = current_node;
            }

            if (!edge.child->visited)
            {
                Q.push(edge.child);
                edge.child->visited = true;
            }
        }
    }

    return _extract_path(tree, current_node);
}

std::vector<NodePtr> shortest_path(Tree& tree, std::vector<NodePtr> start_nodes, std::vector<NodePtr> goal_nodes)
{
    std::priority_queue<NodePtr, std::vector<NodePtr>, compareNodesFunction> Q;

    for (auto start_node : start_nodes)
    {
        start_node->dist = start_node->cost;
        Q.push(start_node);
    }
    NodePtr current_node{ nullptr };
    std::vector<Edge> nb;

    while (!Q.empty())
    {
        //    current_node = Q.front();
        current_node = Q.top();
        Q.pop();

        // is the current node a goal node?
        if (std::find(goal_nodes.begin(), goal_nodes.end(), current_node) != goal_nodes.end())
        // if (current_node == dummy_goal)
        {
            break;
        }

        if (tree.find(current_node) == tree.end())
        {
            continue;
        }
        else
        {
            nb = tree[current_node];
        }

        for (auto edge : nb)
        {
            if (current_node->dist + edge.cost + edge.child->cost < edge.child->dist)
            {
                edge.child->dist = current_node->dist + edge.cost + edge.child->cost;
                edge.child->parent = current_node;
            }

            if (!edge.child->visited)
            {
                Q.push(edge.child);
                edge.child->visited = true;
            }
        }
    }

    return _extract_path(tree, current_node);
}

std::ostream& operator<<(std::ostream& os, const Node& node)
{
    os << "Node: (";
    for (const auto value : node.data)
    {
        os << value << ", ";
    }
    os << ")";
    return os;
}

std::ostream& operator<<(std::ostream& os, const Edge& edge)
{
    os << "Edge: " << *edge.child << " : " << edge.cost;
    return os;
}

std::ostream& operator<<(std::ostream& os, const std::vector<Edge>& edges)
{
    for (const auto& edge : edges)
    {
        os << edge << ", ";
    }
    return os;
}

}  // namespace arf
