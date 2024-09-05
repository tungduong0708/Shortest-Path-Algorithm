#include <iostream>
#include <vector>
#include <limits>

using namespace std;

const int INF = 1000000000;

void floydWarshall(int n, vector<vector<int>> &adj) {
    vector<vector<int>> distance(n + 1, vector<int>(n + 1, INF));

    for (int i = 1; i <= n; i++) {
        for (int j = 1; j <= n; j++) {
            if (i == j) {
                distance[i][j] = 0;
            } else if (adj[i][j]) {
                distance[i][j] = adj[i][j];
            } else {
                distance[i][j] = INF;
            }
        }
    }

    for (int k = 1; k <= n; k++) {
        for (int i = 1; i <= n; i++) {
            for (int j = 1; j <= n; j++) {
                if (distance[i][k] != INF && distance[k][j] != INF) {
                    distance[i][j] = min(distance[i][j], distance[i][k] + distance[k][j]);
                }
            }
        }
    }

    cout << "All-pairs shortest distances:" << endl;
    for (int i = 1; i <= n; i++) {
        for (int j = 1; j <= n; j++) {
            if (distance[i][j] == INF)
                cout << "INF ";
            else
                cout << distance[i][j] << " ";
        }
        cout << endl;
    }
}

int main() {
    int n, m;
    
    cout << "Enter the number of vertices and edges: ";
    cin >> n >> m;

    vector<vector<int>> adj(n + 1, vector<int>(n + 1, 0));

    cout << "Enter the edges (u, v, w):" << endl;
    for (int i = 0; i < m; i++) {
        int u, v, w;
        cin >> u >> v >> w;
        adj[u][v] = w;
    }

    floydWarshall(n, adj);

    return 0;
}
