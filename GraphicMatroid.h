#ifndef K_FORESTS_GRAPHICMATROID_H
#define K_FORESTS_GRAPHICMATROID_H


#include <vector>
#include <memory>
#include <unordered_set>


struct Edge {
public:
    int forest;
    // the index of a forest this edge is in, -1 if not covered
    // forest are enumerated from 0
    // TODO maybe use std::optional<int> instead of the magical constant "-1"

    Edge(int from, int to) : from_(from), to_(to) {
        forest = -1;
    }

    int AnotherVertex(int vertex) const {
        if (vertex == to_) {
            return from_;
        }
        return to_;
    }

    std::pair<int, int> Vertices() {
        return {from_, to_};
    }

private:
    const int from_, to_;
};

class GraphicMatroid {
public:
    explicit GraphicMatroid(const std::vector<std::vector<int>> &adj_list);

    std::vector<std::vector<std::pair<int, int>>> GetForests();

    void GenerateKForests(int k);

    void PrintGraph();

private:
    std::vector<std::vector<std::shared_ptr<Edge>>> adj_list_;   // vertices are enumerated from 0
    int num_forests;

    void DrawNextForestDFS();

    void DFSNextForest(int vertex, std::vector<bool> &visited_vertices);

    void DrawNextForestBFS();

    std::shared_ptr<Edge> FindOutEdge(const std::shared_ptr<Edge> &edge, int forest_index,
                                      const std::unordered_set<std::shared_ptr<Edge>>& allowed_edges);

    bool FindPathAndAugment(const std::shared_ptr<Edge> &initial_edge,
                            std::unordered_set<std::shared_ptr<Edge>> &unvisited_edges);

    int EdgeIsJoining(const std::shared_ptr<Edge> &edge);

    bool TryToAugment();

    std::unordered_set<std::shared_ptr<Edge>> EdgeSet();
};


#endif //K_FORESTS_GRAPHICMATROID_H
