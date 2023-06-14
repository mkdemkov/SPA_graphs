#include <vector>
#include <queue>
#include <unordered_map>

using namespace std;

/**
 * алгоритм Дейкстры
 * @param graph граф в виде списка смежности
 * @param x конечная вершина
 * @return пара значений - время работы в наносек и найденный кратчайший путь
 */
pair<int64_t, int64_t> dijkstra(vector<vector<pair<int, int>>> &graph, int x) {
    auto start = std::chrono::high_resolution_clock::now();
    int n = graph.size();
    vector<int> dist(n, INT_MAX);
    dist[0] = 0;

    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
    pq.push({0, 0});

    while (!pq.empty()) {
        int u = pq.top().second;
        int d = pq.top().first;
        pq.pop();

        if (u == x) {
            auto end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
            return make_pair<int64_t, int64_t>(duration.count(), d);
        }

        if (d > dist[u]) {
            continue;
        }

        for (auto &neighbor : graph[u]) {
            int v = neighbor.first;
            int weight = neighbor.second;

            if (d + weight < dist[v]) {
                dist[v] = d + weight;
                pq.push({dist[v], v});
            }
        }
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
    return make_pair<int64_t, int64_t>(duration.count(), -1);;  // Если путь до вершины x не найден
}

/**
 * алгоритм Беллмана-Форда
 * @param graph граф в виде списка смежности
 * @param x конечная вершина
 * @return пара значений - время работы в наносек и найденный кратчайший путь
 */
pair<int64_t, int64_t> bellmanFord(vector<vector<pair<int, int>>> &graph, int x) {
    auto start = std::chrono::high_resolution_clock::now();
    int n = graph.size();
    vector<int> dist(n, INT_MAX);
    dist[0] = 0;

    for (int i = 1; i < n; ++i) {
        for (int u = 0; u < n; ++u) {
            for (auto &neighbor : graph[u]) {
                int v = neighbor.first;
                int weight = neighbor.second;

                if (dist[u] != INT_MAX && dist[u] + weight < dist[v]) {
                    dist[v] = dist[u] + weight;
                }
            }
        }
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
    return make_pair<int64_t, int64_t>(duration.count(), dist[x]);;
}

/**
 * алгоритм Флойда-Уоршелла
 * @param graph граф
 * @param x конечная вершина
 * @return пара значений - время работы в наносек и найденный кратчайший путь
 */
pair<int64_t, int64_t> floydWarshall(vector<vector<int>> &graph, int x) {
    auto start = std::chrono::high_resolution_clock::now();
    int n = graph.size();
    vector<vector<int>> dist(graph);

    for (int k = 0; k < n; ++k) {
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                if (dist[i][k] != INT_MAX && dist[k][j] != INT_MAX &&
                    dist[i][k] + dist[k][j] < dist[i][j]) {
                    dist[i][j] = dist[i][k] + dist[k][j];
                }
            }
        }
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
    return make_pair<int64_t, int64_t>(duration.count(), dist[0][x]);
}

/**
 * алгоритм A*
 * @param graph граф в виде списка смежности
 * @param x конечная вершина
 * @return пара значений - время работы в наносек и найденный кратчайший путь
 */
pair<int64_t, int64_t> astar(vector<vector<pair<int, int>>> &graph, int x) {
    auto start = chrono::high_resolution_clock::now();
    int n = graph.size();
    vector<int> dist(n, numeric_limits<int>::max());
    dist[0] = 0;

    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
    pq.push({0, 0});

    while (!pq.empty()) {
        int u = pq.top().second;
        int d = pq.top().first;
        pq.pop();

        if (u == x) {
            auto end = chrono::high_resolution_clock::now();
            auto duration = chrono::duration_cast<chrono::nanoseconds>(end - start);

            return make_pair(duration.count(), d);
        }

        if (d > dist[u]) {
            continue;
        }

        for (auto &neighbor : graph[u]) {
            int v = neighbor.first;
            int weight = neighbor.second;

            int heuristic = 0;  // допустимая эвретическая функция
            int newDist = d + weight + heuristic;

            if (newDist < dist[v]) {
                dist[v] = newDist;
                pq.push({newDist, v});
            }
        }
    }

    auto end = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::nanoseconds>(end - start);

    // Если путь до вершины x не найден
    return make_pair(duration.count(), -1);
}