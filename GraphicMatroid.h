#ifndef K_FORESTS_GRAPHICMATROID_H
#define K_FORESTS_GRAPHICMATROID_H


#include <vector>
#include <memory>
#include <unordered_set>
#include "DisjointSets.h"
#include "Edge.h"
#include "LinkCutTree.h"

class GraphicMatroid {
public:
    explicit GraphicMatroid(const std::vector<std::vector<int>> &adj_list);

    std::vector<std::vector<std::pair<int, int>>> GetForests();

    void GenerateKForests(int k);

    void PrintGraph();

private:
    std::vector<std::vector<std::shared_ptr<Edge>>> adj_list_;   // vertices are enumerated from 0
    int num_forests;
    std::vector<DisjointSets> disjoint_components;
    std::vector<LinkCutTree> forests;

    void DrawNextForestDFS();

    void DFSNextForest(int vertex, std::vector<bool> &visited_vertices);

    void DrawNextForestBFS();

    std::shared_ptr<Edge> FindOutEdge(const std::shared_ptr<Edge> &edge, int forest_index,
                                      const std::unordered_set<std::shared_ptr<Edge>> &allowed_edges);

    bool FindPathAndAugment(const std::shared_ptr<Edge> &initial_edge,
                            std::unordered_set<std::shared_ptr<Edge>> &unvisited_edges);

    int EdgeIsJoining(const std::shared_ptr<Edge> &edge);

    bool EdgeIsJoiningForForest(const std::shared_ptr<Edge> &edge, int forest_index);

    bool TryToAugment();

    std::unordered_set<std::shared_ptr<Edge>> EdgeSet();

    std::vector<std::shared_ptr<Edge>> EdgeVector();

    std::tuple<int, std::vector<std::shared_ptr<Edge>>> Layers();

    std::tuple<bool, std::vector<std::unordered_set<std::shared_ptr<Edge>>>> LayersCyclic();

    bool BlockFlowIndependence();

    int BlockFlowIndependenceCyclic();

    void AugmentPath(const std::vector<std::shared_ptr<Edge>> &path, int final_color);

    void InitializeDisjointSets();

    void InitializeForests();
};


#endif //K_FORESTS_GRAPHICMATROID_H
