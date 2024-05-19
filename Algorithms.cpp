//ID:318691821
//email:elichaiza@gmail.com

#include <queue>
#include <iostream>
#include <stdexcept>
#include <vector>
#include<stack>
#include <utility>
#include "Graph.hpp"
#include "Algorithms.hpp"
#include <functional>
#include <limits>


using namespace std;
using namespace ariel;


//the algorithem BFS
bool Algorithms::BFS(vector<vector<int>> matrix, int start_node)
{
    //
    vector<bool> arrive(matrix.size(), false);

    queue<int> queue;

    queue.push(start_node);
    arrive[(size_t)start_node] = true;

    while (!queue.empty())
    {
        int current = queue.front();
        queue.pop();

        for (int i = 0; i < matrix[(size_t)current].size(); i++)
        {
            if (matrix[(size_t)current][(size_t)i] != 0 && !arrive[(size_t)i])
            {
                queue.push(i);
                arrive[(size_t)i] = true;
            }
        }
    }
    for (size_t i = 0; i < arrive.size(); i++)
    {
        if (!arrive[(size_t)i])
        {
            return false;
        }
    }
    return true;
}

void Algorithms::DFS(const vector<vector<size_t>> &graph, size_t startkodkod, vector<bool> &visited) {
    size_t size_of_kodkods = graph.size();
    std::stack<size_t> stack;
    stack.push(startkodkod);

    while (!stack.empty()) {
        size_t currentKodkod = stack.top();
        stack.pop();

        if (!visited[currentKodkod]) {
            visited[currentKodkod] = true;
            std::cout << "Visited kodkod: " << currentKodkod << std::endl;

            for (size_t i = 0; i < size_of_kodkods; ++i) {
                if (graph[currentKodkod][i] != 0 && !visited[i]) {
                    stack.push(i);
                }
            }
        }
    }
}


//cheking if the graph is connected
bool Algorithms::isConnected(Graph &graph)
{
    size_t number_vertices = graph.matrix.size();
    if (number_vertices == 0 || number_vertices == 1) // assuming
    {
        return true;
    }

    else if (!graph.directed)
    {
        return BFS(graph.matrix, 0);
    }

    else
    {
        bool One_Direction = BFS(graph.matrix, 0);
        Graph graphTranspose = graph.getTranspose();
        bool Second_Direction = BFS(graphTranspose.matrix, 0);
        return One_Direction && Second_Direction;
    }
}
//the dijkstra algorithem:
pair<vector<int>, vector<int>> dijkstra(const Graph& g, size_t src) {
    size_t size_of_kodkod = g.matrix.size();
    vector<int> distances(size_of_kodkod, INF);
    vector<int> parents(size_of_kodkod, -1);


    std::priority_queue<pair<int, size_t>, vector<pair<int, size_t>>, std::greater<pair<int, int>>> pq;

    // initialize source vertex
    distances[src] = 0;
    pq.push(std::make_pair(0, src));

    while (!pq.empty()) {
        // get the vertex with the smallest distance
        size_t u = pq.top().second;
        pq.pop();

        // for all the neighbor of U:
        for (size_t v = 0; v < size_of_kodkod; v++) {
            if (g.matrix[u][v] != 0) {
                // relax the edge
                int currDist = distances[u] + g.matrix[u][v];
                if (currDist < distances[v]) {
                    distances[v] = currDist;
                    parents[v] = u;
                    pq.push(std::make_pair(distances[v], v));
                }
            }
        }
    }

    return make_pair(distances, parents);
}
//the Bellman-Ford algorithm to find the shortest path.
pair<vector<int>, vector<int>> Algorithms::BLF2(const Graph &g, size_t src) {
    return BLF(g, src, g.directed);
}
//the Bellman-Ford algorithm to find the shortest path.
pair<vector<int>, vector<int>> Algorithms::BLF(const Graph &graph, size_t src, bool isDirected) {
    size_t n = graph.matrix.size();
    vector<int> distances(n, INF);
    vector<int> parents(n, (int) -1);

    distances[src] = 0;
    // relax all edges n-1 times
    for (size_t i = 0; i < n - 1; i++) {
        bool a = false;  // if we dont relax any edge in the current iteration, then we can break the loop
        // for each edge (u, v) in the graph
        for (size_t u = 0; u < n; u++) {
            for (size_t v = 0; v < n; v++) {
                // if there is an edge between u and v
                if (graph.matrix[u][v] != 0) {
                    // if the graph is undirected, we should ignore the edge that connects the current vertex to its parent
                    if (!isDirected && parents[u] == (size_t) v) {
                        if (distances[u] + graph.matrix[u][v] < distances[v]) {
                        }
                        continue;
                    }

                    // relax the edge (u, v)
                    if (distances[u] == INF || graph.matrix[u][v] == INF) {
                        continue;
                    }

                    if (distances[u] + graph.matrix[u][v] < distances[v]) {
                        distances[v] = distances[u] + graph.matrix[u][v];
                        parents[v] = (size_t) u;
                        a = true;
                    }
                }
            }
        }
        if (!a) {
            break;
        }
    }

    // check for negative-weight cycles
    for (size_t u = 0; u < n; u++) {
        for (size_t v = 0; v < n; v++) {
            if (graph.matrix[u][v] != 0) {
                // if the graph is undirected, we should ignore the edge that connects the current vertex to its parent
                if (!isDirected && parents[u] == (size_t) v) {
                    continue;
                }

                if (distances[u] == INF || graph.matrix[u][v] == INF) {
                    continue;
                }

                if (distances[u] + graph.matrix[u][v] < distances[v]) {
                    parents[v] = (size_t) u;
                    //throw Algorithms::NegativeCycleException(v, parents);
                }
            }
        }
    }

    return std::make_pair(distances, parents);
}

pair<vector<int>, vector<int>> bfs(const Graph &g, size_t src) {
    size_t n = g.matrix.size();

    // init the distances and parents vectors
    vector<int> distances(n, INF);
    vector<int> parents(n, (int) -1);

    // add the source vertex to the queue and set its distance to 0
    distances[src] = 0;
    std::queue<size_t> q;
    q.push(src);

    while (!q.empty()) {
        size_t u = q.front();
        q.pop();
        for (size_t v = 0; v < n; v++) {
            if (g.matrix[u][v] != 0) {
                if (distances[v] == INF) {  // if the vertex is not discovered yet
                    distances[v] = distances[u] + 1;
                    parents[v] = (size_t) u;
                    q.push(v);
                }
            }
        }
    }

    return {distances, parents};
}

//fid the shortest path from a point to another:
string Algorithms::shortestPath(const Graph& graph, size_t src, size_t dest) {
    // check for valid source and destination vertices
    if (src >= graph.matrix.size() || dest >= graph.matrix.size()) {
        //if the starting point is bigger form the number of the vertex so throw error
        //cheking if the starting point is also the destination point
        throw std::invalid_argument("Invalid source or destination vertex");
    }

    if (src == dest) {
        return std::to_string(src);
    }
//here we will be put the vertex that we need to pass to arive to the destination
    pair<vector<int>, vector<int>> shortest_path_result;

    // if the graph is not weighted, we can use BFS to find the shortest path
    if (!graph.weighted) {
        shortest_path_result = bfs(graph, src);
    } else if (graph.negative_wight) {  // if the graph has negative edge weights, we can use Bellman-Ford algorithm
        try {
            shortest_path_result = BLF2(graph, src);
            if (!graph.directed) {  // if the graph is undirected, we can try to find the shortest path from the destination to the source
                pair<vector<int>, vector<int>> bellmanResult2 = BLF2(graph, dest);

                // choose the correct result (the longer path)
                vector<int> parents1 = shortest_path_result.second;
                size_t path1Length = 0;
                int parent = parents1[dest];
                while (parent != -1) {
                    path1Length++;
                    parent = parents1[(size_t)parent];
                }

                vector<int> parents2 = bellmanResult2.second;
                size_t path2Length = 0;
                parent = parents2[src];
                while (parent != -1) {
                    path2Length++;
                    parent = parents2[(size_t)parent];
                }

                if (path1Length < path2Length) {  // update the result if needed
                    shortest_path_result = bellmanResult2;

                    // swap the source and destination vertices
                    dest = src;
                    // create the path from the source to the destination
                    vector<int> one = shortest_path_result.second;
                    string path = std::to_string(dest);
                    reverse(path.begin(), path.end());
                    int parent = one[dest];
                    while (parent != -1) {
                        string temp = std::to_string(parent);
                        reverse(temp.begin(), temp.end());
                        path.insert(0, temp + ">-");
                        parent = one[(size_t)parent];
                    }
                    // reverse the path
                    reverse(path.begin(), path.end());
                    return path;
                }
            }
        } catch (NegativeCycleException& e) {
            return e.what();
        }
    } else {  // if the graph has non-negative edge weights, we can use Dijkstra's algorithm
        shortest_path_result = dijkstra(graph, src);
    }

    // get the shortest path from the src to the dest
    vector<int> distances = shortest_path_result.first;
    vector<int> parents = shortest_path_result.second;

    // if the distance to the destination vertex is infinity, then there is no path between the source and destination vertices
    if (distances[dest] == INF) {
        return "-1";
    }

    // create the path from the source to the destination
    string path = std::to_string(dest);
    int parent = parents[dest];
    while (parent != -1) {
        path.insert(0, std::to_string(parent) + "->");
        parent = parents[(size_t)parent];
    }
    return path;
}


bool Algorithms::dfsWithCycleDetection(const vector<vector<size_t>> &graph, size_t currentNode, vector<bool> &visited) {
    if (visited[currentNode]) {
        return true; // נמצא צומת שכבר ביקרנו בו, מה שמצביע על מעגל
    }

    visited[currentNode] = true;

    for (size_t i = 0; i < graph.size(); ++i) {
        if (graph[currentNode][i] != 0 && dfsWithCycleDetection(graph, i, visited)) {
            return true; // נמצא מעגל בענף הנוכחי
        }
    }

    visited[currentNode] = false; // סיימנו לבקר בענף הנוכחי
    return false;
}


//Check if the graph contains a cycle and return the cycle path if found.

string Algorithms::isContainsCycle(Graph &g) {
    vector<Color> colors((size_t) g.matrix.size(), WHITE);
    vector<size_t> parents((size_t) g.matrix.size(), (size_t) -1);
    vector<size_t> path;

    for (size_t i = 0; i < g.matrix.size(); i++) {
        if (colors[i] == WHITE) {
            string cycle = isContainsCycleUtil(g, i, &colors, &parents, &path);
            if (!cycle.empty()) {
                return cycle;
            }
        }
    }
    return "0";// we dont have a cycle
}


//Utility function to recursively check if the graph contains a cycle starting from a given source vertex

string Algorithms::isContainsCycleUtil(Graph &g, size_t src, vector<Color> *colors, vector<size_t> *parents,
                                       vector<size_t> *path) {
    (*colors)[src] = GRAY;
    path->push_back(src);

    for (size_t v = 0; v < g.matrix.size(); v++) {
        if (g.matrix[src][v] != 0) {
            if ((*colors)[v] == WHITE) {
                (*parents)[v] = src;
                string cycle = isContainsCycleUtil(g, v, colors, parents, path);
                if (!cycle.empty()) {
                    return cycle;
                }
            } else if ((*colors)[v] == GRAY) {
                // Check if it's a directed graph and the current vertex is the parent of the source vertex
                if (!g.directed && (*parents)[src] == (int) v) {
                    continue;
                }

                return constructCyclePath(*path, v);
            }
        }
    }

    (*colors)[src] = BLACK;
    path->pop_back();
    return ""; // No cycle found
}


 // Construct the cycle path from the current traversal path.

string Algorithms::constructCyclePath(vector<size_t> &path, size_t start) {
    string cycle;
    size_t v = 0;

    for (v = 0; v < path.size(); v++) {
        if (path[v] == start) {
            break;
        }
    }

    for (size_t i = v; i < path.size(); i++) {
        cycle += std::to_string(path[i]) + "->";
    }

    cycle += std::to_string(start);
    return cycle;
}

// פונקציה פנימית המשמשת ל-DFS לבדיקת דו-חלוקה וחלוקה לרכיבים
bool Algorithms::dfsForBipartiteCheckWithComponents(const vector<vector<size_t>> &graph, size_t currentNode,
                                                    vector<size_t> &colors, vector<size_t> &currentComponent) {
    if (colors[currentNode] == 1) {
        // הגענו לצומת בצבע השני שכבר ביקרנו בו, מה שמצביע על קצה בין צמתים מאותו צבע
        return false;
    }

    colors[currentNode] = 0; // משייכים את הצומת לצבע הראשון
    currentComponent.push_back(currentNode); // מוסיפים את הצומת לרכיב הנוכחי

    for (size_t i = 0; i < graph.size(); ++i) {
        if (graph[currentNode][i] != 0) {
            if (colors[i] == -1) {
                // צומת שכנים לא משויך, ממשיכים בסריקה
                if (!dfsForBipartiteCheckWithComponents(graph, i, colors, currentComponent)) {
                    return false;
                }
            } else if (colors[i] == 0) {
                // הגענו לצומת בצבע הראשון שכבר ביקרנו בו, מה שמצביע על קצה בין צמתים מאותו צבע
                return false;
            }
        }
    }

    return true;
}


bool Algorithms::negativeCycle(const vector<vector<size_t>> &graph) {
    size_t n = graph.size(); // מספר הצמתים
    vector<size_t> dist(n, 555555); // וקטור לאחסון מרחקים

    // הפעלת האלגוריתם
    for (size_t i = 0; i < n - 1; ++i) {
        for (size_t u = 0; u < n; ++u) {
            for (size_t v = 0; v < n; ++v) {
                size_t weight = graph[u][v]; // משקל הצלע מ-u ל-v

                if (dist[u] != 555555 && dist[v] > dist[u] + weight) {
                    dist[v] = dist[u] + weight;
                }
            }
        }
    }

    // בדיקה למעגל שלילי
    for (size_t u = 0; u < n; ++u) {
        for (size_t v = 0; v < n; ++v) {
            size_t weight = graph[u][v]; // משקל הצלע מ-u ל-v

            if (dist[u] != 555555 && dist[v] > dist[u] + weight) {
                return true; // נמצא מעגל שלילי
            }
        }
    }

    return false; // לא נמצא מעגל שלילי
}

void Algorithms::isBipartite(Graph &g) {
    if (isBipartiteCheck(g.matrix)) {
        std::cout << "graph is Bipartite\n"
                  << std::endl;
    } else {
        std::cout << "graph is not Bipartite\n"
                  << std::endl;
    }
}

/**
 * Check if a given graph is Bipartite using BFS traversal.
 */
bool Algorithms::isBipartiteCheck(std::vector<vector<int>> g) {
    // Create a color array to store colors assigned to all vertices.
    // Vertex number is used as index in this array. The value '-1' of colorArr[i] is used to
    // indicate that no color is assigned to vertex 'i'.
    // The value 1 is used to indicate first color is assigned and value 0 indicates second color is assigned.
    size_t V = g.size();
    size_t colorArr[V];
    for (size_t i = 0; i < V; ++i)
        colorArr[i] = (size_t) -1;

    // This code is to handle disconnected graphs
    for (size_t i = 0; i < V; i++)
        if (colorArr[i] == -1)
            if (!isBipartiteUtil(g, i, colorArr))
                return false;

    return true;
}

bool Algorithms::isBipartiteUtil(vector<vector<int>> &graph, size_t src, size_t colorArr[]) {
    size_t V = graph.size();
    colorArr[src] = 1;

    // Create a queue (FIFO) of vertex numbers and enqueue source vertex for BFS traversal
    queue<size_t> q;
    q.push(src);

    // Run while there are vertices in queue (Similar to BFS)
    while (!q.empty()) {
        // Dequeue a vertex from queue
        size_t u = q.front();
        q.pop();

        // Find all non-colored adjacent vertices
        for (size_t v = 0; v < V; ++v) {
            // An edge from u to v exists and destination v is not colored
            if (graph[(size_t) u][(size_t) v] != 0 && colorArr[v] == -1) {
                // Assign alternate color to this adjacent v of u
                colorArr[v] = 1 - colorArr[u];
                q.push(v);
            }
                // An edge from u to v exists and destination v is colored with same color as u
            else if (graph[(size_t) u][(size_t) v] != 0 && colorArr[v] == colorArr[u])
                return false;
        }
    }

    // If we reach here, then all adjacent vertices can be colored with alternate color
    return true;
}
