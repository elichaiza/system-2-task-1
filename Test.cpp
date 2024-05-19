//ID:318691821
//email:elichaiza@gmail.com

#include "doctest.h"
#include "Algorithms.hpp"
#include "Graph.hpp"
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>
using namespace std;

TEST_CASE("Test isConnected")
{
    ariel::Graph g;
    vector<vector<int>> graph = {
            {0, 1, 0},
            {1, 0, 1},
            {0, 1, 0}};
    g.loadGraph(graph);
    CHECK(ariel::Algorithms::isConnected(g) == true);

    vector<vector<int>> graph2 = {
            {0, 1, 1, 0, 0},
            {1, 0, 1, 0, 0},
            {1, 1, 0, 1, 0},
            {0, 0, 1, 0, 0},
            {0, 0, 0, 0, 0}};
    g.loadGraph(graph2);
    CHECK(ariel::Algorithms::isConnected(g) == false);

    vector<vector<int>> graph3 ={
            {0, 1, 1, 0},
            {1, 0, 1, 1},
            {1, 1, 0, 1},
            {0, 1, 1, 0}
    };
    g.loadGraph(graph3);
    CHECK(ariel::Algorithms::isConnected(g) == true);


    vector<vector<int>> graph4 ={
        {0, 1, 1, 0, 0},
        {1, 0, 1, 1, 0},
        {1, 1, 0, 0, 1},
        {0, 1, 0, 0, 1},
        {0, 0, 1, 1, 0}
        };
    g.loadGraph(graph4);
    CHECK(ariel::Algorithms::isConnected(g) == true);

    vector<vector<int>> graph5 ={
            {0, 1, 0, 0, 0},
            {1, 0, 0, 0, 0},
            {0, 0, 0, 1, 1},
            {0, 0, 1, 0, 1},
            {0, 0, 1, 1, 0}
    };
    g.loadGraph(graph5);
    CHECK(ariel::Algorithms::isConnected(g) == false);
    };


TEST_CASE("Test shortestPath")
{
    ariel::Graph g;
    vector<vector<int>> graph = {
        {0, 1, 0},
        {1, 0, 1},
        {0, 1, 0}};
    g.loadGraph(graph);
    CHECK(ariel::Algorithms::shortestPath(g, 0, 2) == "0->1->2");

    vector<vector<int>> graph2 = {
        {0, 1, 1, 0, 0},
        {1, 0, 1, 0, 0},
        {1, 1, 0, 1, 0},
        {0, 0, 1, 0, 0},
        {0, 0, 0, 0, 0}};
    g.loadGraph(graph2);
    CHECK(ariel::Algorithms::shortestPath(g, 0, 4) == "-1");
    vector<vector<int>> graph3 = {
            {0, 1, 0, 1, 0, 0},
            {1, 0, 1, 0, 1, 0},
            {0, 1, 0, 0, 1, 1},
            {1, 0, 0, 0, 1, 0},
            {0, 1, 1, 1, 0, 1},
            {0, 0, 1, 0, 1, 0}
};
    g.loadGraph(graph3);
    CHECK(ariel::Algorithms::shortestPath(g, 0, 5) == "0->1->2->5");

    vector<vector<int>> graph4 = {
            {0, 1, 1, 0, 0, 0},
            {1, 0, 1, 1, 0, 0},
            {1, 1, 0, 1, 0, 1},
            {0, 1, 1, 0, 1, 0},
            {0, 0, 0, 1, 0, 1},
            {0, 0, 1, 0, 1, 0}
    };
    g.loadGraph(graph4);
    CHECK(ariel::Algorithms::shortestPath(g, 0, 5) == "0->2->5");

    vector<vector<int>> graph5 = {
            {0, 1, 0, 0, 1, 0}, // קשרים של קודקוד 0
            {1, 0, 1, 0, 1, 0}, // קשרים של קודקוד 1
            {0, 1, 0, 1, 0, 1}, // קשרים של קודקוד 2
            {0, 0, 1, 0, 1, 0}, // קשרים של קודקוד 3
            {1, 1, 0, 1, 0, 1}, // קשרים של קודקוד 4
            {0, 0, 1, 0, 1, 0}
    };
    g.loadGraph(graph5);
    CHECK(ariel::Algorithms::shortestPath(g, 1, 5) == "1->2->5");
}
TEST_CASE("Test isContainsCycle")
{
    ariel::Graph g;
    vector<vector<int>> graph = {
        {0, 1, 0},
        {1, 0, 1},
        {0, 1, 0}};
    g.loadGraph(graph);
    CHECK(ariel::Algorithms::isContainsCycle(g) == "0");

    vector<vector<int>> graph2 = {
        {0, 1, 1, 0, 0},
        {1, 0, 1, 0, 0},
        {1, 1, 0, 1, 0},
        {0, 0, 1, 0, 0},
        {0, 0, 0, 0, 0}};
    g.loadGraph(graph2);
    CHECK(ariel::Algorithms::isContainsCycle(g) == "0->1->2->0");

    vector<vector<int>> graph3 = {
            {0, 1, 0, 0, 1}, // קשרים של קודקוד 0
            {1, 0, 1, 0, 0}, // קשרים של קודקוד 1
            {0, 1, 0, 1, 0}, // קשרים של קודקוד 2
            {0, 0, 1, 0, 1}, // קשרים של קודקוד 3
            {1, 0, 0, 1, 0}};
    g.loadGraph(graph3);
    CHECK(ariel::Algorithms::isContainsCycle(g) == "0->1->2->3->4->0");

    vector<vector<int>> graph4 = {
            {0, 1, 0, 0, 0, 1},
            {1, 0, 1, 0, 0, 0},
            {0, 1, 0, 1, 0, 0},
            {0, 0, 1, 0, 1, 0},
            {0, 0, 0, 1, 0, 1},
            {1, 0, 0, 0, 1, 0}};
    g.loadGraph(graph4);
    CHECK(ariel::Algorithms::isContainsCycle(g) == "0->1->2->3->4->5->0");

    vector<vector<int>> graph5 = {
            {0, 1, 0, 0, 0},
            {1, 0, 1, 0, 0},
            {0, 1, 0, 1, 0},
            {0, 0, 1, 0, 1},
            {0, 0, 0, 1, 0}};
    g.loadGraph(graph5);
    CHECK(ariel::Algorithms::isContainsCycle(g) == "0");

}
TEST_CASE("Test isBipartite")
{
    ariel::Graph g;
    vector<vector<int>> graph = {
        {0, 1, 0},
        {1, 0, 1},
        {0, 1, 0}};
    g.loadGraph(graph);
    CHECK(ariel::Algorithms::isBipartiteCheck(g.matrix) == true);

    vector<vector<int>> graph2 = {
        {0, 1, 1, 0, 0},
        {1, 0, 1, 0, 0},
        {1, 1, 0, 1, 0},
        {0, 0, 1, 0, 0},
        {0, 0, 0, 0, 0}};
    g.loadGraph(graph2);
    CHECK(ariel::Algorithms::isBipartiteCheck(g.matrix) == false);

    vector<vector<int>> graph3 = {
        {0, 1, 2, 0, 0},
        {1, 0, 3, 0, 0},
        {2, 3, 0, 4, 0},
        {0, 0, 4, 0, 5},
        {0, 0, 0, 5, 0}};
    g.loadGraph(graph3);
    CHECK(ariel::Algorithms::isBipartiteCheck(g.matrix) == false);

    vector<vector<int>> graph4 = {
            {0, 0, 2, 0, 0},
            {0, 0, 1, 0, 0},
            {2, 1, 0, 4, 0},
            {0, 0, 4, 0, 5},
            {0, 0, 0, 5, 0}};
    g.loadGraph(graph4);
    CHECK(ariel::Algorithms::isBipartiteCheck(g.matrix) == true);

    vector<vector<int>> graph5 = {
            {0, 1, 2, 0, 0},
            {1, 0, 3, 0, 0},
            {2, 3, 0, 2, 0},
            {0, 0, 2, 0, 1},
            {0, 0, 0, 1, 0}};
    g.loadGraph(graph5);
    CHECK(ariel::Algorithms::isBipartiteCheck(g.matrix) == false);

    vector<vector<int>> graph6 = {
            {0, 1, 2, 0, 0},
            {1, 0, 3, 0, 0},
            {2, 3, 0, 4, 0},
            {0, 0, 4, 0, 5},
            {0, 0, 0, 5, 0}};
    g.loadGraph(graph6);
    CHECK(ariel::Algorithms::isBipartiteCheck(g.matrix) == false);
}
TEST_CASE("Test invalid graph")
{
    ariel::Graph g;
    vector<vector<int>> graph1 = {
            {0, 1, 2, 0},
            {1, 0, 3, 0},
            {2, 3, 0, 4},
            {0, 0, 4, 0},
            {0, 0, 0, 5}};
    CHECK_THROWS(g.loadGraph(graph1));

    vector<vector<int>> graph2 = {
            {0, 1, 2, 0,1},
            {0, 0, 3, 0,1},
            {2, 2, 0, 4,1},
            {0, 0, 4, 0,1},
            {0, 0, 0, 5,0}};
    CHECK_THROWS(g.loadGraph(graph2));

    vector<vector<int>> graph3 = {
            {0, 1, 2, 0,0},
            {1, 0, 3, 0,0},
            {2, 3, 0, 4,0},
            {0, 0, 4, 0,0},
            {0, 1, 0, 0,0}};
    CHECK_THROWS(g.loadGraph(graph3));

    vector<vector<int>> graph4 = {
            {0, 1, 2, 0,1},
            {1, 0, 3, 0,1},
            {2, 3, 1, 4,1},
            {0, 0, 4, 0,1},
            {0, 0, 0, 5,0}};
    CHECK_THROWS(g.loadGraph(graph4));
}