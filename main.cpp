#include <iostream>
#include <fstream>
#include "graph.cpp"
#include "algorithms.cpp"

/**
 * функция преобразования списка смежности для алгоритма Флойда-Уоршелла
 * @param graph_pairs начальный список смежности
 * @return преобразованный список смежности
 */
vector<vector<int>> convertGraph(const vector<vector<pair<int, int>>> &graph_pairs) {
    int n = graph_pairs.size();
    vector<vector<int>> graph(n, vector<int>(n, INT_MAX));

    for (int u = 0; u < n; ++u) {
        graph[u][u] = 0;
        for (const auto &neighbor : graph_pairs[u]) {
            int v = neighbor.first;
            int weight = neighbor.second;
            graph[u][v] = weight;
        }
    }

    return graph;
}

/**
 * функция преобразования enum в строку
 * @param type элемент enum
 * @return элемент enum как строка
 */
string getTypeAsString(TypeOfGraph type) {
    switch (type) {

        case COMPLETE:return "Полный";
        case CONNECTED:return "Связный";
        case SPARSE:return "Разреженный";
    }
}

/**
 * Основная функция проведения измерений
 * @param graph граф в виде списка смежности
 * @param type тип графа
 * @param vertexes число вершин
 * @param edges число ребер
 */
void process(vector<vector<pair<int, int>>> graph, TypeOfGraph type, int vertexes, int edges) {
    // откроем таблицы
    ofstream table("../tables/time_edges_vertexes.csv", ios::app);
    string type_string = getTypeAsString(type);
    // 5 раз прогоним каждый алгоритм
    int64_t total = 0;
    int64_t path;
    for (int i = 0; i < 5; ++i) {
        auto res = dijkstra(graph, vertexes / 2);
        total += res.first;
        path = res.second;
    }
    auto time = total / 5; // это усредненное время после 5 прогонов
    // выводим полученные значения в csv
    table << type_string << ";" << vertexes << ";" << edges << ";" << "Дейкстра;" << time << ";"
          << path << "\n";

    total = 0;
    for (int i = 0; i < 5; ++i) {
        auto res = bellmanFord(graph, vertexes / 2);
        total += res.first;
        path = res.second;
    }
    time = total / 5; // это усредненное время после 5 прогонов
    // выводим полученные значения в csv
    table << type_string << ";" << vertexes << ";" << edges << ";" << "Беллман-Форд;" << time << ";"
          << path << "\n";

    total = 0;
    auto updated_graph = convertGraph(graph);
    for (int i = 0; i < 5; ++i) {
        auto res = floydWarshall(updated_graph, vertexes / 2);
        total += res.first;
        path = res.second;
    }
    time = total / 5; // это усредненное время после 5 прогонов
    // выводим полученные значения в csv
    table << type_string << ";" << vertexes << ";" << edges << ";" << "Флойд-Уоршелл;" << time
          << ";"
          << path << "\n";

    total = 0;
    for (int i = 0; i < 5; ++i) {
        auto res = astar(graph, vertexes / 2);
        total += res.first;
        path = res.second;
    }
    time = total / 5; // это усредненное время после 5 прогонов
    // выводим полученные значения в csv
    table << type_string << ";" << vertexes << ";" << edges << ";" << "A*;" << time << ";"
          << path << "\n";
}

int main() {
    ofstream table("../tables/time_edges_vertexes.csv");
    table << "Graph_type;Vertexes;Edges;Algorithm;Time;Path" << "\n";
    table.close();
    Graph graph; // создадим объект Graph
    auto *types = new TypeOfGraph[3]{COMPLETE, CONNECTED, SPARSE};
    // пройдемся по всем типам графов
    for (int i = 0; i < 3; ++i) {
        // пройдемся по всем размерам графа
        for (int vertexes = 10; vertexes <= 1010; vertexes += 50) {
            // построили граф
            auto gr = graph.buildGraph(types[i], vertexes);
            auto edges = graph.countEdges();
            process(gr, types[i], vertexes, edges);
        }
    }
    return 0;
}
