//ID:318691821
//email:elichaiza@gmail.com

#ifndef ALGORITHMS_HPP
#define ALGORITHMS_HPP
#include <queue>
#include <iostream>
#include <stdexcept>
#include <vector>
#include <algorithm>
#include "Graph.hpp"
using namespace std;
enum Color
{
    WHITE,
    GRAY,
    BLACK
};
#define INF std::numeric_limits<int>::max()  // the infinity
#define NO_EDGE INF                          // represent no edge between two vertices as infinity

namespace ariel{

    class Algorithms{

    public:
        void DFS(const vector<vector<size_t>>& graph, size_t startNode, vector<bool>& visited);


        static bool isConnected(Graph &graph);
        static bool BFS(vector<vector<int >> matrix, int start_node);

        static string shortestPath(const Graph& g, size_t src, size_t dest);

        static bool dfsWithCycleDetection(const vector<vector<size_t >>& graph, size_t currentNode, vector<bool>& visited);

        static string isContainsCycle(Graph &graph);
        static string isContainsCycleUtil(Graph &g, size_t src, vector<Color> *colors, vector<size_t> *parents, vector<size_t> *path);
        static string constructCyclePath(vector<size_t> &path, size_t start);



        bool dfsForBipartiteCheckWithComponents(const vector<vector<size_t>>& graph, size_t currentNode, vector<size_t>& colors, vector<size_t>& currentComponent);

        bool negativeCycle(const vector<vector<size_t>>& graph) ;

        static pair<vector<int>, vector<int>> BLF(const Graph& g, size_t src, bool isDirected);

        static pair<vector<int>, vector<int>> BLF2(const Graph& g, size_t src);

        static void isBipartite(Graph &g);
        static bool isBipartiteCheck(std::vector<vector<int>> g);
        static bool isBipartiteUtil(vector<vector<int>> &graph, size_t src, size_t colorArr[]);
        ;};

class NegativeCycleException : public std::exception {
public:
    vector<size_t> cycle;

    NegativeCycleException(size_t detectedCycleStart, vector<int> parentList) {
        int cycleVertices = detectedCycleStart;

        // make sure that we in the cycle
        for (size_t i = 0; i < parentList.size(); i++) {
            cycleVertices = parentList[(size_t)cycleVertices];
        }

        vector<size_t> cycle;

        for (size_t v = (size_t)cycleVertices; true; v = (size_t)parentList[v]) {
            cycle.push_back(v);
            if (v == cycleVertices && cycle.size() > 1) {
                break;
            }
        }

        reverse(cycle.begin(), cycle.end());

        this->cycle = cycle;
    }

    virtual const char* what() const throw() {
        return "Graph contains a negative-weight cycle";
    }
};
}

#endif