#include <stack>
#include <iostream>
#include <queue>
#include <unordered_map>
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
    for (int i = 0; i < k; ++i) {
        DrawNextForestBFS();
    }

    while (TryToAugment()) {};
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

std::vector<std::shared_ptr<Edge>> GraphicMatroid::ExchangeGraphNeighbors(
        const std::shared_ptr<Edge> &edge, int forest_index) {
    // returns the fundamental cycle of edge in the tree_index'th forest
    // TODO this is O(m), rewrite

    std::vector<std::shared_ptr<Edge>> ans;

    if (edge->forest == forest_index) {
        return ans;
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
        return ans;
    }

    int path_vertex = final_vertex;
    while (path_vertex != initial_vertex) {
        ans.push_back(edge_to_parent[path_vertex]);
        path_vertex = edge_to_parent[path_vertex]->AnotherVertex(path_vertex);
    }
    return ans;
}

bool GraphicMatroid::FindPathAndAugment(const std::shared_ptr<Edge> &initial_edge,
                                        std::unordered_set<std::shared_ptr<Edge>> &visited_edges) {
    // finds an augmenting path and augments, if the path exists
    // returns true if successful, false otherwise

    if (initial_edge->forest != -1) {
        return false;
    }

    std::queue<std::shared_ptr<Edge>> queue;
    queue.push(initial_edge);

    std::unordered_map<std::shared_ptr<Edge>, std::shared_ptr<Edge>> parents;
    visited_edges.insert(initial_edge);

    int final_forest_index = -1;
    std::shared_ptr<Edge> final_edge;
    while ((!queue.empty()) && (final_forest_index == -1)) {
        auto current_edge = queue.front();
        queue.pop();

        for (int forest_index = 0; forest_index < num_forests; ++forest_index) {
            for (const auto &next_edge: ExchangeGraphNeighbors(current_edge, forest_index)) {

                if (visited_edges.find(next_edge) != visited_edges.end()) {
                    continue;
                }

                queue.push(next_edge);
                parents[next_edge] = current_edge;
                visited_edges.insert(next_edge);

                final_forest_index = EdgeIsJoining(next_edge);
                if (final_forest_index != -1) {
                    final_edge = next_edge;
                    break;
                }
            }
        }
    }

    if (final_forest_index == -1) {
        return false;
    }

    int current_color = final_forest_index;
    while (final_edge != initial_edge) {
        int tmp_color = final_edge->forest;
        final_edge->forest = current_color;
        current_color = tmp_color;
        final_edge = parents[final_edge];
    }
    initial_edge->forest = current_color;

    return true;
}

int GraphicMatroid::EdgeIsJoining(const std::shared_ptr<Edge> &edge) {
    // if the edge is joining, returns the index of the corresponding forest
    // else returns -1

    for (int forest_index = 0; forest_index < num_forests; ++forest_index) {
        if (edge->forest == forest_index) {
            continue;
        }
        if (ExchangeGraphNeighbors(edge, forest_index).empty()) {
            return forest_index;
        }
    }

    return -1;
}

bool GraphicMatroid::TryToAugment() {
    std::unordered_set<std::shared_ptr<Edge>> visited_edges;

    for (int vertex = 0; vertex < static_cast<int>(adj_list_.size()); ++vertex) {
        for (const auto &edge: adj_list_[vertex]) {
            if (vertex < edge->AnotherVertex(vertex)) {
                if (FindPathAndAugment(edge, visited_edges)) {
                    return true;
                }
            }
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
