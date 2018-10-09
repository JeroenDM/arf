#ifndef _UTIL_H_
#define _UTIL_H_

#include <iostream>

#include "arf_graph/graph.h"

std::ostream& operator<<(std::ostream& os, const Node& n)
{
    using namespace std;
    os << "(" << n.path_index << ", ";
    os << n.sample_index << ")";
    os << " dist: " << n.shortest_distance;
    if (n.parent == nullptr)
    {
        os << " parent: no parent";
    }
    else
    {
        os << " parent: ";
        os << "(" << (*n.parent).path_index << ", ";
        os << (*n.parent).sample_index << ")";
    }
    os << " jv: ( ";
    for (auto q : (*n.jv))
    {
        os << q << ", ";
    }
    os << ")";
    return os;
}

std::ostream& operator<<(std::ostream& os, const Graph& graph)
{
    os << "-----------------------\n";
    os << "Graph\n";
    for (auto path_point_nodes : graph.getNodes())
    {
        os << "-----------------------\n";
        for (auto node : path_point_nodes)
        {
            os << node << "\n";
        }
    }
    os << "-----------------------";
    return os;
}

#endif