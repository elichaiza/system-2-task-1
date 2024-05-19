//ID:318691821
//email:elichaiza@gmail.com

#ifndef GRAPH_HPP
#define GRAPH_HPP
#include <iostream>
#include <stdexcept>
#include <vector>
using namespace std;

namespace ariel{

    class Graph{
    public:
    // the regulars that have to graph
        vector<vector<int>> matrix;
        bool directed;
        bool negative_wight;
        bool weighted;
        size_t vertices;

        Graph(bool directed=false);

        Graph getTranspose();

        void loadGraph(vector<vector<int>> &graph);

        void printGraph();

        size_t getNumberOfVertices();

    };
}
#endif