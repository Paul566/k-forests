#include <stack>
#include <iostream>
#include "GraphicMatroid.h"

GraphicMatroid::GraphicMatroid(const std::vector<std::vector<int>> &adj_list) {
    // adj_list has to have vertices enumerated from 0

    num_forests = 0;
    adj_list_ = std::vector<std::vector<std::shared_ptr<Edge>>>(adj_list.size());

    for (int i = 0; i < static_cast<int>(adj_list.size()); ++i) {
        adj_list_[i].reserve(adj_list[i].size());
        for (int to : adj_list[i]) {
            if (to > i) {
                Edge new_edge(i, to);
                std::shared_ptr<Edge> edge_ptr = std::make_shared<Edge>(new_edge);
                adj_list_[i].push_back(edge_ptr);
                adj_list_[to].push_back(edge_ptr);
            }
        }
    }
}

std::vector<std::vector<std::pair<int, int>>> GraphicMatroid::GetForests()  {
    // returns a vector of forests, each forest represented with an edge list
    std::vector<std::vector<std::pair<int, int>>> ans(num_forests, std::vector<std::pair<int, int>>(0));

    for (int i = 0; i < static_cast<int>(adj_list_.size()); ++i) {
        for (const auto& edge : adj_list_[i]) {
            if ((edge->tree != -1) && (i < edge->AnotherVertex(i))) {
                ans[edge->tree].push_back(edge->Vertices());
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

void GraphicMatroid::DFSNextForest(int vertex, std::vector<bool>& visited_vertices) {
    // performs DFS from the vertex via free edges,
    // takes the edges of the DFS tree into num_forests'th forest

    visited_vertices[vertex] = true;
    for (const auto & edge : adj_list_[vertex]) {
        if ((edge->tree == -1) && (!visited_vertices[edge->AnotherVertex(vertex)])) {
            edge->tree = num_forests;
            DFSNextForest(edge->AnotherVertex(vertex), visited_vertices);
        }
    }
}

void GraphicMatroid::GenerateKForests(int k) {
    // incomplete
    for (int i = 0; i < k; ++i) {
        DrawNextForestDFS();
    }
}

void GraphicMatroid::PrintGraph() {
    std::cout << "Adjacency list:\n";
    for (int vertex = 0; vertex < static_cast<int>(adj_list_.size()); ++vertex) {
        std::cout << vertex << ":";
        for (auto & edge : adj_list_[vertex]) {
            std::cout << " (to: " << edge->AnotherVertex(vertex) << ", tree: " << edge->tree << ")";
        }
        std::cout << "\n";
    }
}
