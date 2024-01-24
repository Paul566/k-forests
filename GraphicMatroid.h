#ifndef K_FORESTS_GRAPHICMATROID_H
#define K_FORESTS_GRAPHICMATROID_H


#include <vector>
#include <memory>


struct Edge {
public:
    int tree;
    // the index of a tree this edge is in, -1 if not covered
    // trees are enumerated from 0
    // TODO maybe use std::optional<int> instead of the magical constant "-1"

    Edge(int from, int to) : from_(from), to_(to) {
        tree = -1;
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
    GraphicMatroid(const std::vector<std::vector<int>> &adj_list);

    std::vector<std::vector<std::pair<int, int>>> GetForests ();

    void GenerateKForests(int k);

    void PrintGraph();

private:
    std::vector<std::vector<std::shared_ptr<Edge>>> adj_list_;   // vertices are enumerated from 0
    int num_forests;

    void DrawNextForestDFS();

    void DFSNextForest(int vertex, std::vector<bool>& visited_vertices);
};


#endif //K_FORESTS_GRAPHICMATROID_H
