#include <iostream>
#include <vector>
#include <queue>
#include <limits>

using namespace std;

const int INF = 1000000000;

void spfa(int V, int s, vector<vector<pair<int, int>>> &AL) {
    vector<int> dist(V, INF);
    dist[s] = 0;

    queue<int> q;
    q.push(s);

    vector<int> in_queue(V, 0);
    in_queue[s] = 1;

    while (!q.empty()) {
        int u = q.front();
        q.pop();
        in_queue[u] = 0;

        for (auto &[v, w] : AL[u]) {
            if (dist[u] + w < dist[v]) {
                dist[v] = dist[u] + w;

                if (!in_queue[v]) {
                    q.push(v);
                    in_queue[v] = 1;
                }
            }
        }
    }

    cout << "Shortest distances from node " << s << ":" << endl;
    for (int i = 0; i < V; i++) {
        if (dist[i] == INF)
            cout << "Node " << i << ": INF" << endl;
        else
            cout << "Node " << i << ": " << dist[i] << endl;
    }
}

int main() {
    int V, E, s;
    
    cout << "Enter number of vertices, number of edges, and source vertex: ";
    cin >> V >> E >> s;

    vector<vector<pair<int, int>>> AL(V);

    cout << "Enter edges (u, v, w):" << endl;
    for (int i = 0; i < E; i++) {
        int u, v, w;
        cin >> u >> v >> w;
        AL[u].push_back({v, w});
    }

    spfa(V, s, AL);

    return 0;
}
