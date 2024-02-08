#include <stack>
#include <iostream>
#include <queue>
#include <unordered_map>
#include <algorithm>
#include "GraphicMatroid.h"

GraphicMatroid::GraphicMatroid(const std::vector<std::vector<int>> &adj_list) {
    // adj_list has to have vertices enumerated from 0

    num_forests = 0;
    adj_list_ = std::vector<std::vector<std::shared_ptr<Edge>>>(adj_list.size());

    for (int i = 0; i < static_cast<int>(adj_list.size()); ++i) {
        adj_list_[i].reserve(adj_list[i].size());
        for (int to: adj_list[i]) {
            if (to > i) {
                Edge new_edge(i, to);
                std::shared_ptr<Edge> edge_ptr = std::make_shared<Edge>(new_edge);
                adj_list_[i].push_back(edge_ptr);
                adj_list_[to].push_back(edge_ptr);
            }
        }
    }
}

std::vector<std::vector<std::pair<int, int>>> GraphicMatroid::GetForests() {
    // returns a vector of forests, each forest represented with an edge list
    std::vector<std::vector<std::pair<int, int>>> ans(num_forests, std::vector<std::pair<int, int>>(0));

    for (int i = 0; i < static_cast<int>(adj_list_.size()); ++i) {
        for (const auto &edge: adj_list_[i]) {
            if ((edge->forest != -1) && (i < edge->AnotherVertex(i))) {
                ans[edge->forest].push_back(edge->Vertices());
            }
        }
    }

    return ans;
}

void GraphicMatroid::DrawNextForestDFS() {
    // takes the edges of the DFS forest via free edges into num_forests'th forest,
    // increases num_forests by 1

    std::vector<bool> visited_vertices(adj_list_.size(), false);

    for (int vertex = 0; vertex < static_cast<int>(adj_list_.size()); ++vertex) {
        if (!visited_vertices[vertex]) {
            DFSNextForest(vertex, visited_vertices);
        }
    }

    ++num_forests;
}

void GraphicMatroid::DFSNextForest(int vertex, std::vector<bool> &visited_vertices) {
    // performs DFS from the vertex via free edges,
    // takes the edges of the DFS tree into num_forests'th forest

    visited_vertices[vertex] = true;
    for (const auto &edge: adj_list_[vertex]) {
        if ((edge->forest == -1) && (!visited_vertices[edge->AnotherVertex(vertex)])) {
            edge->forest = num_forests;
            DFSNextForest(edge->AnotherVertex(vertex), visited_vertices);
        }
    }
}

void GraphicMatroid::GenerateKForests(int k) {
    if (k <= 0) {
        return;
    }

    for (int i = 0; i < k; ++i) {
        DrawNextForestDFS();
    }
    InitializeDisjointSets();
    InitializeForests();

    // PrintGraph();

    while (BlockFlowIndependence()) {};

    // while (BlockFlowIndependenceCyclic() != -1) {};
}

void GraphicMatroid::PrintGraph() {
    std::cout << "Adjacency list:\n";
    for (int vertex = 0; vertex < static_cast<int>(adj_list_.size()); ++vertex) {
        std::cout << vertex << ":";
        for (auto &edge: adj_list_[vertex]) {
            std::cout << " (to: " << edge->AnotherVertex(vertex) << ", forest: " << edge->forest << ")";
        }
        std::cout << "\n";
    }
}

bool GraphicMatroid::FindPathAndAugment(const std::shared_ptr<Edge> &initial_edge,
                                        std::unordered_set<std::shared_ptr<Edge>> &unvisited_edges) {
    // finds an augmenting path and augments, if the path exists
    // returns true if successful, false otherwise

    if (initial_edge->forest != -1) {
        return false;
    }

    std::queue<std::shared_ptr<Edge>> queue;
    queue.push(initial_edge);
    unvisited_edges.erase(initial_edge);

    std::unordered_map<std::shared_ptr<Edge>, std::shared_ptr<Edge>> parents;

    while (!queue.empty()) {
        auto current_edge = queue.front();
        queue.pop();

        for (int forest_index = 0; forest_index < num_forests; ++forest_index) {
            std::shared_ptr<Edge> next_edge = FindOutEdge(current_edge, forest_index, unvisited_edges);
            unvisited_edges.erase(next_edge);
            while (next_edge != nullptr) {
                queue.push(next_edge);
                parents[next_edge] = current_edge;

                int final_forest_index = EdgeIsJoining(next_edge);

                if (final_forest_index != -1) {
                    // found an augmenting path
                    std::vector<std::shared_ptr<Edge>> path;
                    while (next_edge != initial_edge) {
                        path.push_back(next_edge);
                        next_edge = parents[next_edge];
                    }
                    path.push_back(initial_edge);
                    std::reverse(path.begin(), path.end());
                    AugmentPath(path, final_forest_index);
                    return true;
                }

                next_edge = FindOutEdge(current_edge, forest_index, unvisited_edges);
                unvisited_edges.erase(next_edge);
            }
        }
    }

    return false;
}

int GraphicMatroid::EdgeIsJoining(const std::shared_ptr<Edge> &edge) {
    // if the edge is joining, returns the index of the corresponding forest
    // else returns -1

    for (int forest_index = 0; forest_index < num_forests; ++forest_index) {
        if (edge->forest == forest_index) {
            continue;
        }
        if (disjoint_components[forest_index].Representative(edge->Vertices().first) !=
            disjoint_components[forest_index].Representative(edge->Vertices().second)) {
            return forest_index;
        }
    }

    return -1;
}

bool GraphicMatroid::TryToAugment() {
    auto unvisited_edges = EdgeSet();

    auto edge_vector = EdgeVector();
    for (const auto &edge: edge_vector) {
        if (FindPathAndAugment(edge, unvisited_edges)) {
            return true;
        }
    }

    return false;
}

void GraphicMatroid::DrawNextForestBFS() {
    // takes the edges of the BFS forest via free edges into num_forests'th forest,
    // increases num_forests by 1

    std::vector<bool> visited_vertices(adj_list_.size(), false);

    for (int rooting_vertex = 0; rooting_vertex < static_cast<int>(adj_list_.size()); ++rooting_vertex) {
        if (!visited_vertices[rooting_vertex]) {
            std::queue<int> queue;
            queue.push(rooting_vertex);

            visited_vertices[rooting_vertex] = true;

            while (!queue.empty()) {
                int current_vertex = queue.front();
                queue.pop();

                for (const auto &next_edge: adj_list_[current_vertex]) {
                    int next_vertex = next_edge->AnotherVertex(current_vertex);
                    if ((visited_vertices[next_vertex]) || (next_edge->forest != -1)) {
                        continue;
                    }
                    queue.push(next_vertex);
                    visited_vertices[next_vertex] = true;
                    next_edge->forest = num_forests;
                }
            }
        }
    }

    ++num_forests;
}

std::shared_ptr<Edge> GraphicMatroid::FindOutEdge(const std::shared_ptr<Edge> &edge, int forest_index,
                                                  const std::unordered_set<std::shared_ptr<Edge>> &allowed_edges) {
    // returns an edge in a fundamental cycle of the given edge
    // that is in allowed_edges and in the tree_index'th forest,
    // if no such edge exists, returns nullptr
    // TODO this is O(m), rewrite

    if (edge->forest == forest_index) {
        return nullptr;
    }

    int initial_vertex = edge->Vertices().first;
    int final_vertex = edge->Vertices().second;

    std::queue<int> queue;
    queue.push(initial_vertex);

    std::vector<bool> visited_vertices(adj_list_.size(), false);
    std::vector<std::shared_ptr<Edge>> edge_to_parent(adj_list_.size(), edge);
    visited_vertices[initial_vertex] = true;

    bool path_exists = false;
    while ((!queue.empty()) && (!path_exists)) {
        int current_vertex = queue.front();
        queue.pop();

        for (const auto &next_edge: adj_list_[current_vertex]) {
            int next_vertex = next_edge->AnotherVertex(current_vertex);

            if ((next_edge->forest != forest_index) || (visited_vertices[next_vertex])) {
                continue;
            }

            queue.push(next_vertex);
            edge_to_parent[next_vertex] = next_edge;
            visited_vertices[next_vertex] = true;
            if (next_vertex == final_vertex) {
                path_exists = true;
                break;
            }
        }
    }

    if (!path_exists) {
        return nullptr;
    }

    int path_vertex = final_vertex;
    while (path_vertex != initial_vertex) {
        if (allowed_edges.find(edge_to_parent[path_vertex]) != allowed_edges.end()) {
            return edge_to_parent[path_vertex];
        }
        path_vertex = edge_to_parent[path_vertex]->AnotherVertex(path_vertex);
    }
    return nullptr;
}

std::unordered_set<std::shared_ptr<Edge>> GraphicMatroid::EdgeSet() {
    // returns unordered_set of pointers for all edges
    std::unordered_set<std::shared_ptr<Edge>> edge_set;

    for (const auto &list: adj_list_) {
        for (const auto &edge: list) {
            edge_set.insert(edge);
        }
    }

    return edge_set;
}

std::tuple<bool, std::vector<std::shared_ptr<Edge>>> GraphicMatroid::Layers() {
    // updates layers of edges, returns true if T is reachable
    // also returns uncovered edges

    std::vector<std::vector<std::shared_ptr<Edge>>> layers;

    std::vector<std::shared_ptr<Edge>> free_edges;
    auto edge_vector = EdgeVector();
    int edges_limit = static_cast<int>(edge_vector.size());
    for (const auto &edge: edge_vector) {
        if (edge->forest == -1) {
            free_edges.push_back(edge);
        } else {
            forests[edge->forest].UpdateEdgeLevel(edge, INT32_MAX);
        }
    }
    layers.push_back(free_edges);

    for (int counter = 0; counter < edges_limit; ++counter) {
        std::vector<std::shared_ptr<Edge>> next_layer;
        for (const auto &edge: layers.back()) {
            if (EdgeIsJoining(edge) != -1) {
                return std::forward_as_tuple(true, free_edges);
            }

            for (int forest_index = 0; forest_index < num_forests; ++forest_index) {
                auto next_level_edge = forests[forest_index].MaxLevelEdge(edge->Vertices().first,
                                                                          edge->Vertices().second);
                while (next_level_edge != nullptr) {
                    if (next_level_edge->level < INT32_MAX) {
                        break;
                    }
                    next_layer.push_back(next_level_edge);
                    forests[forest_index].UpdateEdgeLevel(next_level_edge, static_cast<int>(layers.size()));
                    next_level_edge = forests[forest_index].MaxLevelEdge(edge->Vertices().first,
                                                                         edge->Vertices().second);
                }
            }
        }

        if (next_layer.empty()) {
            return std::forward_as_tuple(false, free_edges);
        }

        layers.push_back(next_layer);
    }

    return std::forward_as_tuple(false, free_edges);
}

std::vector<std::shared_ptr<Edge>> GraphicMatroid::EdgeVector() {
    // returns vector of all edges

    std::vector<std::shared_ptr<Edge>> edge_vector;

    for (int vertex = 0; vertex < static_cast<int>(adj_list_.size()); ++vertex) {
        for (const auto &edge: adj_list_[vertex]) {
            if (vertex < edge->AnotherVertex(vertex)) {
                edge_vector.push_back(edge);
            }
        }
    }

    return edge_vector;
}

bool GraphicMatroid::BlockFlowIndependence() {
    // BlockFlowIndependence procedure from Terao, "Faster Matroid Partition Algorithms"
    // returns true if success

    auto layers_output = Layers();
    if (!std::get<0>(layers_output)) {
        return false;
    }

    auto uncovered_edges = std::get<1>(layers_output);
    std::vector<std::shared_ptr<Edge>> current_path;
    for (const auto &initial_edge: uncovered_edges) {
        current_path.push_back(initial_edge);
        int next_layer_index = 1;  // next vertex should be in that level

        while (!current_path.empty()) {
            std::shared_ptr<Edge> current_edge = current_path.back();

            int edge_is_joining = EdgeIsJoining(current_edge);
            if (edge_is_joining != -1) {
                AugmentPath(current_path, edge_is_joining);
                current_path.clear();
                next_layer_index = 0;
                break;
            }

            std::shared_ptr<Edge> next_edge = forests[0].MaxLevelEdge(current_edge->Vertices().first,
                                                                      current_edge->Vertices().second);

            for (int forest_index = 1; forest_index < num_forests; ++forest_index) {
                if (next_edge != nullptr) {
                    if (next_edge->level >= next_layer_index) {
                        break;
                    }
                }
                next_edge = forests[forest_index].MaxLevelEdge(current_edge->Vertices().first,
                                                               current_edge->Vertices().second);
            }

            if (next_edge == nullptr) {
                --next_layer_index;
                if (current_edge->forest != -1) {
                    forests[current_edge->forest].UpdateEdgeLevel(current_edge, INT32_MIN);
                }
                current_path.pop_back();
            } else {
                if ((next_edge->level < next_layer_index) || (next_edge->level == INT32_MAX)) {
                    --next_layer_index;
                    if (current_edge->forest != -1) {
                        forests[current_edge->forest].UpdateEdgeLevel(current_edge, INT32_MIN);
                    }
                    current_path.pop_back();
                } else {
                    ++next_layer_index;
                    current_path.push_back(next_edge);
                }
            }
        }
    }

    return true;
}

void GraphicMatroid::AugmentPath(const std::vector<std::shared_ptr<Edge>> &path, int final_color) {
    for (int i = 0; i < static_cast<int>(path.size()) - 1; ++i) {
        forests[path[i + 1]->forest].CutEdge(path[i + 1]);
        path[i]->level = INT32_MIN;
        forests[path[i + 1]->forest].Link(path[i]);
        path[i]->forest = path[i + 1]->forest;
    }
    path.back()->level = INT32_MIN;
    forests[final_color].Link(path.back());
    path.back()->forest = final_color;

    disjoint_components[final_color].Unite(path.back()->Vertices().first, path.back()->Vertices().second);
}

void GraphicMatroid::InitializeDisjointSets() {
    for (int forest_index = 0; forest_index < num_forests; ++forest_index) {
        DisjointSets next_sets(static_cast<int>(adj_list_.size()));
        disjoint_components.push_back(next_sets);
    }

    for (int vertex = 0; vertex < static_cast<int>(adj_list_.size()); ++vertex) {
        for (const auto &edge: adj_list_[vertex]) {
            if ((vertex < edge->AnotherVertex(vertex)) && (edge->forest != -1)) {
                disjoint_components[edge->forest].Unite(vertex, edge->AnotherVertex(vertex));
            }
        }
    }
}

int GraphicMatroid::BlockFlowIndependenceCyclic() {
    // Experimental stuff
    // BlockFlowIndependence procedure but for cyclic paths
    // returns number of layers, -1 if not successful

    auto layers_output = LayersCyclic();
    if (!std::get<0>(layers_output)) {
        return -1;
    }

    auto layers = std::get<1>(layers_output);
    std::vector<std::shared_ptr<Edge>> current_path;
    int next_layer_index = 0;  // next vertex should be in that level
    while (!((layers[0].empty()) && (current_path.empty()))) {
        if (next_layer_index == 0) {
            std::shared_ptr<Edge> next_edge = *layers[0].begin();
            layers[0].erase(next_edge);
            ++next_layer_index;
            current_path.push_back(next_edge);
        } else {
            std::shared_ptr<Edge> current_edge = current_path.back();
            if (next_layer_index == static_cast<int>(layers.size())) {
                // we are at a final level

                if (EdgeIsJoiningForForest(current_edge, (next_layer_index - 1) % num_forests)) {
                    AugmentPath(current_path, (next_layer_index - 1) % num_forests);
                    current_path.clear();
                    next_layer_index = 0;
                } else {
                    --next_layer_index;
                    current_path.pop_back();
                }
            } else {
                std::shared_ptr<Edge> next_edge = FindOutEdge(current_edge, (next_layer_index - 1) % num_forests,
                                                              layers[next_layer_index]);
                if (next_edge == nullptr) {
                    --next_layer_index;
                    current_path.pop_back();
                } else {
                    layers[next_layer_index].erase(next_edge);
                    ++next_layer_index;
                    current_path.push_back(next_edge);
                }
            }
        }
    }

    return static_cast<int>(layers.size());
}

std::tuple<bool, std::vector<std::unordered_set<std::shared_ptr<Edge>>>> GraphicMatroid::LayersCyclic() {
    // Experimental
    // returns ((T is reachable from s), (l_0, ..., l_d)),
    // where l_i is at distance i + 1 from s in a cyclic exchange graph

    std::vector<std::unordered_set<std::shared_ptr<Edge>>> layers;

    std::unordered_set<std::shared_ptr<Edge>> unvisited_edges = EdgeSet();
    int edges_limit = static_cast<int>(unvisited_edges.size());

    std::unordered_set<std::shared_ptr<Edge>> free_edges;
    auto edge_vector = EdgeVector();
    for (const auto &edge: edge_vector) {
        if (edge->forest == -1) {
            free_edges.insert(edge);
            unvisited_edges.erase(edge);
        }
    }
    layers.push_back(free_edges);

    int forest_index = 0;
    for (int counter = 0; counter < edges_limit; ++counter) {
        std::unordered_set<std::shared_ptr<Edge>> next_layer;
        for (const auto &edge: layers.back()) {
            if (EdgeIsJoiningForForest(edge, forest_index)) {
                return std::forward_as_tuple(true, layers);
            }

            auto next_level_edge = FindOutEdge(edge, forest_index, unvisited_edges);
            while (next_level_edge != nullptr) {
                unvisited_edges.erase(next_level_edge);
                next_layer.insert(next_level_edge);
                next_level_edge = FindOutEdge(edge, forest_index, unvisited_edges);
            }
        }

        if (next_layer.empty()) {
            return std::forward_as_tuple(false, layers);
        }

        layers.push_back(next_layer);
        ++forest_index;
        if (forest_index == num_forests) {
            forest_index = 0;
        }
    }

    return std::forward_as_tuple(false, layers);
}

bool GraphicMatroid::EdgeIsJoiningForForest(const std::shared_ptr<Edge> &edge, int forest_index) {
    if (edge->forest == forest_index) {
        return false;
    }
    return (disjoint_components[forest_index].Representative(edge->Vertices().first) !=
            disjoint_components[forest_index].Representative(edge->Vertices().second));
}

void GraphicMatroid::InitializeForests() {
    for (int forest_index = 0; forest_index < num_forests; ++forest_index) {
        LinkCutTree next_forest(static_cast<int>(adj_list_.size()));
        forests.push_back(next_forest);
    }

    for (int vertex = 0; vertex < static_cast<int>(adj_list_.size()); ++vertex) {
        for (const auto &edge: adj_list_[vertex]) {
            if ((vertex < edge->AnotherVertex(vertex)) && (edge->forest != -1)) {
                forests[edge->forest].Link(edge);
            }
        }
    }
}
