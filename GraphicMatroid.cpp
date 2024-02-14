#include <stack>
#include <iostream>
#include <queue>
#include <unordered_map>
#include <algorithm>
#include <utility>
#include "GraphicMatroid.h"

GraphicMatroid::GraphicMatroid(const std::vector<std::vector<int>> &adj_list, int num_forests,
                               std::string initialization_type)
        : initialization_type_(std::move(initialization_type)), num_forests_(num_forests) {
    // adj_list has to have vertices enumerated from 0

    generator = std::mt19937(239);
    num_augmentations = 0;
    shortest_augmentation_length = INT32_MAX;
    longest_augmentation_length = 0;
    num_find_edge_levels = 0;
    num_find_edge_bfi = 0;

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

    for (int forest_index = 0; forest_index < num_forests_; ++forest_index) {
        LinkCutTree next_forest(static_cast<int>(adj_list_.size()) * 2 - 1);
        forests.push_back(next_forest);
        edges_of_forests.emplace_back();
    }
}

std::vector<std::vector<std::pair<int, int>>> GraphicMatroid::GetForests() {
    // returns a vector of forests, each forest represented with an edge list
    std::vector<std::vector<std::pair<int, int>>> ans(num_forests_, std::vector<std::pair<int, int>>(0));

    for (int i = 0; i < static_cast<int>(adj_list_.size()); ++i) {
        for (const auto &edge: adj_list_[i]) {
            if ((edge->forest != -1) && (i < edge->AnotherVertex(i))) {
                ans[edge->forest].push_back(edge->Vertices());
            }
        }
    }

    return ans;
}

void GraphicMatroid::DrawNextForestDFS(int next_forest_index) {
    // takes the edges of the DFS forest via free edges into num_forests'th forest,
    // increases num_forests by 1

    std::vector<bool> visited_vertices(adj_list_.size(), false);

    for (int vertex = 0; vertex < static_cast<int>(adj_list_.size()); ++vertex) {
        if (!visited_vertices[vertex]) {
            DFSNextForest(vertex, visited_vertices, next_forest_index);
        }
    }
}

void GraphicMatroid::DFSNextForest(int vertex, std::vector<bool> &visited_vertices, int next_forest_index) {
    // performs DFS from the vertex via free edges,
    // takes the edges of the DFS tree into num_forests'th forest

    visited_vertices[vertex] = true;
    for (const auto &edge: adj_list_[vertex]) {
        if ((edge->forest == -1) && (!visited_vertices[edge->AnotherVertex(vertex)])) {
            edge->forest = next_forest_index;
            DFSNextForest(edge->AnotherVertex(vertex), visited_vertices, next_forest_index);
        }
    }
}

void GraphicMatroid::InitializeEverything() {
    if (initialization_type_ == "random") {
        InitializeRandomForests();
    } else {
        if (initialization_type_ == "DFS") {
            for (int i = 0; i < num_forests_; ++i) {
                DrawNextForestDFS(i);
            }
        } else {
            if (initialization_type_ == "BFS") {
                for (int i = 0; i < num_forests_; ++i) {
                    DrawNextForestBFS(i);
                }
            } else {
                throw std::runtime_error("unknown initialization type: " + initialization_type_);
            }
        }

        InitializeDisjointSets();
    }

    InitializeForests();
}

void GraphicMatroid::GenerateKForests() {
    if (num_forests_ <= 0) {
        return;
    }

    InitializeEverything();

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

int GraphicMatroid::EdgeIsJoining(const std::shared_ptr<Edge> &edge) {
    // if the edge is joining, returns the index of the corresponding forest
    // else returns -1

    for (int forest_index = 0; forest_index < num_forests_; ++forest_index) {
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

void GraphicMatroid::DrawNextForestBFS(int next_forest_index) {
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
                    next_edge->forest = next_forest_index;
                }
            }
        }
    }
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

bool GraphicMatroid::Layers(std::vector<std::shared_ptr<Edge>> &uncovered_edges, std::vector<int> &layer_sizes) {
    // updates layers of edges, returns true if T is reachable,
    // also updates uncovered edges and sizes of layers

    std::vector<std::shared_ptr<Edge>> previous_layer;

    auto edge_vector = EdgeVector();
    int edges_limit = static_cast<int>(edge_vector.size());
    for (const auto &edge: edge_vector) {
        if (edge->forest == -1) {
            uncovered_edges.push_back(edge);
        } else {
            UpdateEdgeLevel(edge, INT32_MAX);
        }
    }
    layer_sizes.push_back(static_cast<int>(uncovered_edges.size()));
    previous_layer = uncovered_edges;

    for (int counter = 0; counter < edges_limit; ++counter) {
        std::vector<std::shared_ptr<Edge>> next_layer;
        for (const auto &edge: previous_layer) {
            if (EdgeIsJoining(edge) != -1) {
                return true;
            }

            for (int forest_index = 0; forest_index < num_forests_; ++forest_index) {
                auto next_level_edge = FindOutEdge(edge, forest_index);
                ++num_find_edge_levels;

                while (next_level_edge != nullptr) {
                    if (next_level_edge->level < INT32_MAX) {
                        break;
                    }
                    next_layer.push_back(next_level_edge);
                    UpdateEdgeLevel(next_level_edge, static_cast<int>(layer_sizes.size()));
                    next_level_edge = FindOutEdge(edge, forest_index);
                    ++num_find_edge_levels;
                }
            }
        }

        if (next_layer.empty()) {
            return false;
        }

        layer_sizes.push_back(static_cast<int>(next_layer.size()));
        previous_layer = std::move(next_layer);
    }

    return false;
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

    std::vector<std::shared_ptr<Edge>> uncovered_edges;
    std::vector<int> layer_sizes;
    bool success = Layers(uncovered_edges, layer_sizes);
    if (!success) {
        return false;
    }
    int num_layers = static_cast<int>(layer_sizes.size());

    std::vector<std::shared_ptr<Edge>> current_path;
    for (const auto &initial_edge: uncovered_edges) {
        current_path.push_back(initial_edge);
        int next_layer_index = 1;  // next vertex should be in that level
        if (layer_sizes[next_layer_index] <= 0) {
            return true;
        }

        while (!current_path.empty()) {
            std::shared_ptr<Edge> current_edge = current_path.back();

            if (next_layer_index == num_layers) {
                // we are at a final level, if current_edge is not joining, leave it
                int edge_is_joining = EdgeIsJoining(current_edge);
                if (edge_is_joining != -1) {
                    AugmentPath(current_path, edge_is_joining);
                    current_path.clear();
                    next_layer_index = 0;
                    for (int &size: layer_sizes) {
                        --size;
                    }
                    break;
                } else {
                    layer_sizes[next_layer_index - 1] -= 1;
                    if (layer_sizes[next_layer_index - 1] <= 0) {
                        return true;
                    }
                    --next_layer_index;
                    if (current_edge->forest != -1) {
                        UpdateEdgeLevel(current_edge, INT32_MIN);
                    }
                    current_path.pop_back();
                    continue;
                }
            }

            std::shared_ptr<Edge> next_edge = FindOutEdge(current_edge, 0);
            ++num_find_edge_bfi;

            for (int forest_index = 1; forest_index < num_forests_; ++forest_index) {
                if (next_edge != nullptr) {
                    if (next_edge->level >= next_layer_index) {
                        break;
                    }
                }
                next_edge = FindOutEdge(current_edge, forest_index);
                ++num_find_edge_bfi;
            }

            if (next_edge == nullptr) {
                layer_sizes[next_layer_index - 1] -= 1;
                if (layer_sizes[next_layer_index - 1] <= 0) {
                    return true;
                }
                --next_layer_index;
                if (current_edge->forest != -1) {
                    UpdateEdgeLevel(current_edge, INT32_MIN);
                }
                current_path.pop_back();
            } else {
                if ((next_edge->level < next_layer_index) || (next_edge->level == INT32_MAX)) {
                    layer_sizes[next_layer_index - 1] -= 1;
                    if (layer_sizes[next_layer_index - 1] <= 0) {
                        return true;
                    }
                    --next_layer_index;
                    if (current_edge->forest != -1) {
                        UpdateEdgeLevel(current_edge, INT32_MIN);
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
        UpdateEdgeLevel(path[i], INT32_MIN);
        MoveEdge(path[i + 1], path[i]);
    }
    UpdateEdgeLevel(path.back(), INT32_MIN);
    AddEdge(path.back(), final_color);

    ++num_augmentations;
    if (static_cast<int>(path.size()) > longest_augmentation_length) {
        longest_augmentation_length = static_cast<int>(path.size());
    }
    if (static_cast<int>(path.size()) < shortest_augmentation_length) {
        shortest_augmentation_length = static_cast<int>(path.size());
    }
}

void GraphicMatroid::InitializeDisjointSets() {
    for (int forest_index = 0; forest_index < num_forests_; ++forest_index) {
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

bool GraphicMatroid::EdgeIsJoiningForForest(const std::shared_ptr<Edge> &edge, int forest_index) {
    if (edge->forest == forest_index) {
        return false;
    }
    return (disjoint_components[forest_index].Representative(edge->Vertices().first) !=
            disjoint_components[forest_index].Representative(edge->Vertices().second));
}

void GraphicMatroid::InitializeForests() {
    for (int vertex = 0; vertex < static_cast<int>(adj_list_.size()); ++vertex) {
        for (const auto &edge: adj_list_[vertex]) {
            if ((vertex < edge->AnotherVertex(vertex)) && (edge->forest != -1)) {
                AddEdge(edge, edge->forest);
            }
        }
    }
}

int GraphicMatroid::NextRandomIndex(std::vector<int> &indices, int &num_already_drawn) {
    if (num_already_drawn >= static_cast<int>(indices.size())) {
        throw std::runtime_error("in NextRandomIndex: already drawn everything");
    }

    std::uniform_int_distribution<int> distribution(num_already_drawn, static_cast<int>(indices.size()) - 1);
    int answer_index = distribution(generator);
    int answer = indices[answer_index];
    std::swap(indices[answer_index], indices[num_already_drawn]);
    ++num_already_drawn;
    return answer;
}

void GraphicMatroid::InitializeRandomForests() {
    // greedily gets k random forests, also generates DisjointSets

    auto edges = EdgeVector();
    std::shuffle(edges.begin(), edges.end(), generator);

    for (int forest_index = 0; forest_index < num_forests_; ++forest_index) {
        DisjointSets next_sets(static_cast<int>(adj_list_.size()));
        disjoint_components.push_back(next_sets);
    }

    for (const auto &edge: edges) {
        std::vector<int> forest_indices;
        forest_indices.reserve(num_forests_);
        for (int i = 0; i < num_forests_; ++i) {
            forest_indices.push_back(i);
        }
        int num_already_checked = 0;
        for (int i = 0; i < num_forests_; ++i) {
            int forest_index = NextRandomIndex(forest_indices, num_already_checked);
            if (disjoint_components[forest_index].Representative(edge->Vertices().first) !=
                disjoint_components[forest_index].Representative(edge->Vertices().second)) {
                edge->forest = forest_index;
                disjoint_components[forest_index].Unite(edge->Vertices().first, edge->Vertices().second);
                break;
            }
        }
    }
}

std::shared_ptr<Edge> GraphicMatroid::FindOutEdge(const std::shared_ptr<Edge>& edge, int forest_index) {
    int index = forests[forest_index].MaxLevelVertex(edge->Vertices().first, edge->Vertices().second);

    if (index < static_cast<int>(adj_list_.size())) {
        return nullptr;
    }

    return edges_of_forests[forest_index][index - static_cast<int>(adj_list_.size())];
}

void GraphicMatroid::UpdateEdgeLevel(const std::shared_ptr<Edge>& edge, int new_level) {
    edge->level = new_level;
    if (edge->forest != -1) {
        forests[edge->forest].UpdateLevel(static_cast<int>(adj_list_.size()) + edge->index_in_forest, new_level);
    }
}

void GraphicMatroid::MoveEdge(const std::shared_ptr<Edge>& old_edge, const std::shared_ptr<Edge>& new_edge) {
    // in the forest of old_edge, old_edge is deleted and new edge is added
    // an augmentation is a sequence of MoveEdge with an AddEdge in the end

    if (new_edge->forest != -1) {
        throw std::runtime_error("in MoveEdge: new edge has to be empty");
    }

    int forest_index = old_edge->forest;
    int edge_index_in_forest = old_edge->index_in_forest;
    forests[forest_index].CutEdge(old_edge->Vertices().first, static_cast<int>(adj_list_.size()) + edge_index_in_forest);
    forests[forest_index].CutEdge(old_edge->Vertices().second, static_cast<int>(adj_list_.size()) + edge_index_in_forest);
    old_edge->forest = -1;
    old_edge->index_in_forest = -1;

    edges_of_forests[forest_index][edge_index_in_forest] = new_edge;

    forests[forest_index].Link(static_cast<int>(adj_list_.size()) + edge_index_in_forest, new_edge->Vertices().first);
    forests[forest_index].Link(new_edge->Vertices().second, static_cast<int>(adj_list_.size()) + edge_index_in_forest);
    new_edge->forest = forest_index;
    new_edge->index_in_forest = edge_index_in_forest;
}

void GraphicMatroid::AddEdge(const std::shared_ptr<Edge>& edge, int forest_index) {
    int edge_index = static_cast<int>(edges_of_forests[forest_index].size());
    if (edge_index > static_cast<int>(adj_list_.size()) - 2) {
        throw std::runtime_error("in AddEdge: this forest already contains n-1 edges");
    }

    edges_of_forests[forest_index].push_back(edge);

    forests[forest_index].Link(static_cast<int>(adj_list_.size()) + edge_index, edge->Vertices().first);
    forests[forest_index].Link(edge->Vertices().second, static_cast<int>(adj_list_.size()) + edge_index);
    edge->forest = forest_index;
    edge->index_in_forest = edge_index;

    disjoint_components[forest_index].Unite(edge->Vertices().first, edge->Vertices().second);
}
