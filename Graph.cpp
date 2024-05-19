
//ID:318691821
//email:elichaiza@gmail.com
#include "Graph.hpp"
#include <iostream>
#include <stdexcept>
#include <vector>
using namespace std;
namespace ariel{

    // Constructor  to graph
    Graph::Graph(bool directed) {
        this->directed=directed;
        this->negative_wight= false;
        this->weighted= false;
        this->vertices=0;
    }
    //loading a graph and cheaking if the graph is a ligel:
    void Graph::loadGraph(vector<vector<int>>  &graph){
        this->vertices=graph.size();
        matrix = graph;

        // check if a directed graph is a symmetric matrix
        for (size_t i = 0; i < matrix.size(); i++) {
            for (size_t j = 0; j < matrix[i].size(); j++) {
                if (matrix[i][j] != 0) {
                    if (matrix[i][j] != 1) {
                        this->weighted = true;
                    }
                    if (matrix[i][j] < 0) {
                        this->negative_wight = true;
                    }
                }

                if (!this->directed && matrix[i][j] != matrix[j][i]) {
                    throw invalid_argument("Invalid graph: The graph is not symmetric");
                }
            }
    }
          //check if the graph is semetrik graph:
            for(size_t i=0;i<matrix.size(); i++) {
            if(matrix.size()!=matrix[i].size()){
              throw invalid_argument("Invalid graph: The graph is not a square matrix");
            }
            }
    }
    //printing the graph
    void Graph::printGraph(){
        cout << "the graph is:" <<  endl;
        for (size_t i = 0; i <matrix.size(); i++){
            for(size_t p=0; p<matrix.size(); p++){
                int ans=matrix[i][p];
                cout << ans << " " ;
            }
            cout<<"\n"<<endl;
        }
    }

    //doing transpose to the graph
    Graph Graph::getTranspose()
    {
        Graph gTranspose(true);
        size_t num_vertices = this->vertices;
        vector<vector<int> > transpose(num_vertices, vector<int>(num_vertices, 0));
        for (size_t i = 0; i < num_vertices; ++i)
        {
            for (size_t j = 0; j < num_vertices; ++j)
            {
                transpose[j][i] = this->matrix[i][j];
            }
        }
        gTranspose.loadGraph(transpose); // Assign the transposed matrix to GT
        return gTranspose;
    }
     //getin the number of the vertices of the graph we have
    size_t Graph::getNumberOfVertices(){
        return matrix.size();
    }

    ;}

