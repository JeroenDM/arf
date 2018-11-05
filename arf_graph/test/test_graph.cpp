#include <gtest/gtest.h>
#include <iostream>

#include "arf_graph/graph.h"
#include "arf_graph/util.h"

const bool VERBOSE = false;

TEST(Node, createNode)
{
    JointPose jv;
    Node n;
    EXPECT_TRUE(true);
}

class TestGraph : public ::testing::Test
{
  protected:
    void SetUp() override
    {
        jp1 = { { 0, 0 }, { 0, 1 } };
        jp2 = { { 1, -1 }, { 1, 0 }, { 1, 1 } };
        jp3 = { { 0, 2 }, { 2, 2 } };
        g1.addPathPointData(jp1);
        g1.addPathPointData(jp2);
        g1.addPathPointData(jp3);
    }

    std::vector<JointPose> jp1, jp2, jp3;
    Graph g1;
};

TEST_F(TestGraph, addPathPointData)
{
    Graph g;
    g.addPathPointData(jp1);
    g.addPathPointData(jp2);
    g.addPathPointData(jp3);

    EXPECT_TRUE(true);
    if (VERBOSE) std::cout << g << std::endl;
}

TEST_F(TestGraph, nonDefaultConstructor)
{
    std::vector<std::vector<JointPose>> data = { jp1, jp2, jp3 };
    Graph g(data);

    EXPECT_TRUE(true);
    if (VERBOSE) std::cout << g << std::endl;
}

TEST_F(TestGraph, getNeighbors)
{
    Node n{};
    n.path_index = 1;
    std::vector<Node*> nb = g1.getNeighbors(&n);

    EXPECT_TRUE(true);
    if (VERBOSE)
    {
        for (auto node : nb)
        {
            std::cout << (*node) << std::endl;
        }
    }
}

TEST_F(TestGraph, dijkstra)
{
    g1.runMultiSourceDijkstra();
    std::vector<Node*> sp = g1.getShortestPath();

    EXPECT_TRUE(true);
    if (VERBOSE)
    {
        std::cout << g1 << std::endl;
        std::cout << "Shortest path \n";
        for (auto node : sp)
        {
            std::cout << (*node) << std::endl;
        }
    }
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}