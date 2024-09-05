#include <iostream>
#include <vector>
#include <tuple>
#include <limits>

using namespace std;

const int INF = 1000000000;

void bellmanFord(int n, int m, int x, vector<tuple<int, int, int>> &edges) {
    vector<int> distance(n + 1, INF);
    distance[x] = 0;

    // Relax edges up to n-1 times
    for (int i = 1; i <= n - 1; i++) {
        for (auto e : edges) {
            int a, b, w;
            tie(a, b, w) = e;

            if (distance[a] != INF && distance[a] + w < distance[b]) {
                distance[b] = distance[a] + w;
            }
        }
    }

    for (auto e : edges) {
        int a, b, w;
        tie(a, b, w) = e;
        if (distance[a] != INF && distance[a] + w < distance[b]) {
            cout << "Negative weight cycle detected!" << endl;
            return;
        }
    }

    cout << "Shortest distances from node " << x << ":" << endl;
    for (int i = 1; i <= n; i++) {
        if (distance[i] == INF)
            cout << "Node " << i << ": INF" << endl;
        else
            cout << "Node " << i << ": " << distance[i] << endl;
    }
}

int main() {
    int n, m, x;
    
    cout << "Enter number of nodes, number of edges, and source node: ";
    cin >> n >> m >> x;

    vector<tuple<int, int, int>> edges;

    cout << "Enter edges (a, b, w): " << endl;
    for (int i = 0; i < m; i++) {
        int a, b, w;
        cin >> a >> b >> w;
        edges.push_back({a, b, w});
    }

    bellmanFord(n, m, x, edges);

    return 0;
}
