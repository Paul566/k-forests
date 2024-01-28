#include <stdexcept>
#include <chrono>
#include "Tester.h"
#include "GraphicMatroid.h"

Tester::Tester(const std::vector<std::vector<int>> &adj_list, const std::vector<int> &ground_truth_sizes) :
        adj_list_(adj_list), ground_truth_sizes_(ground_truth_sizes) {
    if (adj_list.size() + 1 != ground_truth_sizes.size()) {
        throw std::runtime_error("the size of the answers list is not n + 1");
    }
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
    // TODO
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

