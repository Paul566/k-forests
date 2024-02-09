#include <stdexcept>
#include <chrono>
#include <queue>
#include "Tester.h"
#include "GraphicMatroid.h"

Tester::Tester(const std::vector<std::vector<int>> &adj_list, const std::vector<int> &ground_truth_sizes) :
        adj_list_(adj_list), ground_truth_sizes_(ground_truth_sizes) {
}

double Tester::RunTest(int k) {
    // runs a test for a given k, throws an error in case of something sus
    // returns time (excluding initialization) in seconds

    GraphicMatroid graph(adj_list_);

    auto start = std::chrono::high_resolution_clock::now();
    graph.GenerateKForests(k);
    auto stop = std::chrono::high_resolution_clock::now();

    auto forests = graph.GetForests();
    if (!ForestsAreDisjoint(forests)) {
        throw std::runtime_error("forests are not disjoint");
    }
    if (adj_list_.size() + 1 != ground_truth_sizes_.size()) {
        throw std::runtime_error("the size of the answers list is not n + 1");
    }
    if (ground_truth_sizes_[k] != SumOfSizes(forests)) {
        throw std::runtime_error("size is wrong");
    }
    for (const auto &forest: forests) {
        if (!IsAcyclic(forest)) {
            throw std::runtime_error("a forest is not acyclic");
        }
    }

    double duration = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
            stop - start).count()) / 1'000'000;
    return duration;
}

bool Tester::ForestsAreDisjoint(const std::vector<std::vector<std::pair<int, int>>> &forests) {
    std::vector<int64_t> edge_hashes;
    for (const auto& forest : forests) {
        for (auto edge : forest) {
            int from = edge.first;
            int to = edge.second;
            if (from > to) {
                to = edge.first;
                from = edge.second;
            }
            edge_hashes.push_back(from * static_cast<int64_t>(adj_list_.size()) + to);
        }
    }

    std::unordered_set<int64_t> edge_set;
    for (int64_t hash : edge_hashes) {
        if (edge_set.find(hash) == edge_set.end()) {
            edge_set.insert(hash);
        } else {
            return false;
        }
    }

    return true;
}

int Tester::SumOfSizes(const std::vector<std::vector<std::pair<int, int>>> &forests) {
    int size = 0;
    for (const auto& forest: forests) {
        size += static_cast<int>(forest.size());
    }
    return size;
}

bool Tester::IsAcyclic(const std::vector<std::pair<int, int>> &forest) {
    std::vector<std::vector<int>> forest_adj_list(adj_list_.size(), std::vector<int>());
    for (auto edge : forest) {
        forest_adj_list[edge.first].push_back(edge.second);
        forest_adj_list[edge.second].push_back(edge.first);
    }

    std::vector<bool> visited_vertices(forest_adj_list.size(), false);
    std::vector<int> parents(forest_adj_list.size(), -1);

    for (int rooting_vertex = 0; rooting_vertex < static_cast<int>(forest_adj_list.size()); ++rooting_vertex) {
        if (!visited_vertices[rooting_vertex]) {
            std::queue<int> queue;
            queue.push(rooting_vertex);

            visited_vertices[rooting_vertex] = true;

            while (!queue.empty()) {
                int current_vertex = queue.front();
                queue.pop();

                for (int next_vertex: forest_adj_list[current_vertex]) {
                    if (visited_vertices[next_vertex]) {
                        if (next_vertex != parents[current_vertex]) {
                            return false;
                        }
                        continue;
                    }
                    queue.push(next_vertex);
                    visited_vertices[next_vertex] = true;
                    parents[next_vertex] = current_vertex;
                }
            }
        }
    }

    return true;
}

std::vector<double> Tester::RunForAllK() {
    std::vector<double> times;
    times.reserve(ground_truth_sizes_.size());

    for (int k = 0; k < static_cast<int>(ground_truth_sizes_.size()); ++k) {
        times.push_back(RunTest(k));
    }

    return times;
}

std::pair<double, int> Tester::GetTimeAndSize(int k) {
    // returns (time, size_of_the_answer)
    // does not run any checks

    GraphicMatroid graph(adj_list_);

    auto start = std::chrono::high_resolution_clock::now();
    graph.GenerateKForests(k);
    auto stop = std::chrono::high_resolution_clock::now();

    auto forests = graph.GetForests();
    double duration = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
            stop - start).count()) / 1'000'000;
    return {duration, SumOfSizes(forests)};
}

