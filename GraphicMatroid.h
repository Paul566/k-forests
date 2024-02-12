#ifndef K_FORESTS_GRAPHICMATROID_H
#define K_FORESTS_GRAPHICMATROID_H


#include <vector>
#include <memory>
#include <unordered_set>
#include <random>
#include "DisjointSets.h"
#include "Edge.h"
#include "LinkCutTree.h"

class GraphicMatroid {
public:
    // statistics gathering
    int num_augmentations;
    int shortest_augmentation_length;
    int longest_augmentation_length;
    int num_find_edge_levels;
    int num_find_edge_bfi;  // number of "find next edge of maximal level" in BlockFlowIndependence;

    explicit GraphicMatroid(const std::vector<std::vector<int>> &adj_list, int num_forests,
                            std::string initialization_type);

    std::vector<std::vector<std::pair<int, int>>> GetForests();

    void GenerateKForests();

    void PrintGraph();

    int NextRandomIndex(std::vector<int> &indices, int &num_already_drawn);


private:
    std::vector<std::vector<std::shared_ptr<Edge>>> adj_list_;   // vertices are enumerated from 0
    const std::string initialization_type_;   // BFS or DFS
    const int num_forests_;
    std::vector<DisjointSets> disjoint_components;
    std::vector<LinkCutTree> forests;
    std::mt19937 generator;

    void DrawNextForestDFS(int next_forest_index);

    void DFSNextForest(int vertex, std::vector<bool> &visited_vertices, int next_forest_index);

    void DrawNextForestBFS(int next_forest_index);

    std::shared_ptr<Edge> FindOutEdge(const std::shared_ptr<Edge> &edge, int forest_index,
                                      const std::unordered_set<std::shared_ptr<Edge>> &allowed_edges);

    bool FindPathAndAugment(const std::shared_ptr<Edge> &initial_edge,
                            std::unordered_set<std::shared_ptr<Edge>> &unvisited_edges);

    int EdgeIsJoining(const std::shared_ptr<Edge> &edge);

    bool EdgeIsJoiningForForest(const std::shared_ptr<Edge> &edge, int forest_index);

    bool TryToAugment();

    std::unordered_set<std::shared_ptr<Edge>> EdgeSet();

    std::vector<std::shared_ptr<Edge>> EdgeVector();

    bool Layers(std::vector<std::shared_ptr<Edge>>& uncovered_edges, std::vector<int>& layer_sizes);

    std::tuple<bool, std::vector<std::unordered_set<std::shared_ptr<Edge>>>> LayersCyclic();

    bool BlockFlowIndependence();

    int BlockFlowIndependenceCyclic();

    void AugmentPath(const std::vector<std::shared_ptr<Edge>> &path, int final_color);

    void InitializeDisjointSets();

    void InitializeForests();

    void InitializeEverything();

    void InitializeRandomForests();
};


#endif //K_FORESTS_GRAPHICMATROID_H
