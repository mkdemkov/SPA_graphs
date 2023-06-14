#include <iostream>
#include <vector>
#include "static/enum.cpp"
#include <random>

using namespace std;

/**
 * Класс для основный функций с графом
 */
class Graph {

 public:
    /**
     * функция для построения графа заданного типа с заданным числом вершин
     * @param type_of_graph тип графа
     * @param n число вершин
     * @return граф в виде списка смежности
     */
    vector<vector<pair<int, int>>> buildGraph(TypeOfGraph type_of_graph, int n) {
        srand(time(nullptr));
        vector<vector<pair<int, int>>> graph(n);
        switch (type_of_graph) {
            case COMPLETE: {
                for (int i = 0; i < n; ++i) {
                    for (int j = i + 1; j < n; ++j) {
                        int weight = rand() % 10 + 1;
                        graph[i].push_back({j, weight});
                        graph[j].push_back({i, weight});
                    }
                }
                break;
            }
            case CONNECTED: {
                random_device rd;
                mt19937 gen(rd());
                uniform_real_distribution<float> dis(0.0, 1.0);
                float density = 0.4;
                for (int i = 0; i < n; ++i) {
                    for (int j = i + 1; j < n; ++j) {
                        float random_value = dis(gen);

                        if (random_value <= density) {
                            int weight = rand() % 10 + 1;
                            graph[i].push_back({j, weight});
                            graph[j].push_back({i, weight});
                        }
                    }
                }
                break;
            }
            case SPARSE: {
                random_device rd;
                mt19937 gen(rd());
                uniform_int_distribution<int> dis(1, 11);
                for (int i = 1; i < n; ++i) {
                    int weight = dis(gen); // Генерируем случайный вес ребра
                    int parent = i / 2; // Выбираем случайного родителя

                    graph[parent].push_back({i,
                                             weight}); // Добавляем ребро от родителя к текущей вершине
                    graph[i].push_back({parent,
                                        weight}); // Добавляем обратное ребро от текущей вершины к родителю
                }
                break;
            }
        }
        graph_ = graph;
        return graph;
    }

    /**
     * функция для подсчета числа ребер в графе
     * @return число ребер
     */
    int countEdges() {
        int edgeCount = 0;

        for (const auto &neighbors : graph_) {
            edgeCount += static_cast<int>(neighbors.size());
        }

        return edgeCount / 2;
    }

 private:
    vector<vector<pair<int, int>>> graph_;
};