#include <iostream>
#include <vector>
#include <queue>
#include <climits>

using namespace std;

typedef pair<int, int> iPair;

// Dijkstra's algorithm using priority queue (min-heap)
void dijkstra(vector<vector<iPair>>& graph, int src, int V) {
    // Priority queue to store (distance, vertex)
    priority_queue<iPair, vector<iPair>, greater<iPair>> pq;

    vector<int> dist(V, INT_MAX);  // Distance from source to each vertex

    pq.push(make_pair(0, src));  // Push source to priority queue
    dist[src] = 0;

    while (!pq.empty()) {
        int u = pq.top().second;
        pq.pop();

        for (auto& neighbor : graph[u]) {
            int v = neighbor.first;
            int weight = neighbor.second;

            if (dist[v] > dist[u] + weight) {
                dist[v] = dist[u] + weight;
                pq.push(make_pair(dist[v], v));  // Push updated distance to the priority queue
            }
        }
    }

    // Print the result
    cout << "Vertex \t Distance from Source\n";
    for (int i = 0; i < V; i++)
        cout << i << " \t\t " << dist[i] << endl;
}

int main() {
    int V = 9;
    vector<vector<iPair>> graph(V);

    // Add edges to the graph (u, v, weight)
    graph[0].push_back(make_pair(1, 4));
    graph[0].push_back(make_pair(7, 8));
    graph[1].push_back(make_pair(0, 4));
    graph[1].push_back(make_pair(2, 8));
    graph[1].push_back(make_pair(7, 11));
    graph[2].push_back(make_pair(1, 8));
    graph[2].push_back(make_pair(3, 7));
    graph[2].push_back(make_pair(8, 2));
    graph[2].push_back(make_pair(5, 4));
    graph[3].push_back(make_pair(2, 7));
    graph[3].push_back(make_pair(4, 9));
    graph[3].push_back(make_pair(5, 14));
    graph[4].push_back(make_pair(3, 9));
    graph[4].push_back(make_pair(5, 10));
    graph[5].push_back(make_pair(2, 4));
    graph[5].push_back(make_pair(3, 14));
    graph[5].push_back(make_pair(4, 10));
    graph[5].push_back(make_pair(6, 2));
    graph[6].push_back(make_pair(5, 2));
    graph[6].push_back(make_pair(7, 1));
    graph[6].push_back(make_pair(8, 6));
    graph[7].push_back(make_pair(0, 8));
    graph[7].push_back(make_pair(1, 11));
    graph[7].push_back(make_pair(6, 1));
    graph[7].push_back(make_pair(8, 7));
    graph[8].push_back(make_pair(2, 2));
    graph[8].push_back(make_pair(6, 6));
    graph[8].push_back(make_pair(7, 7));

    dijkstra(graph, 0, V);  // Call the function with source vertex as 0
    return 0;
}

