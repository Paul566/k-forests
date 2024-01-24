#include <iostream>
#include <vector>
#include "GraphicMatroid.h"

int main() {
    std::vector<std::vector<int>> adj_list(8);

    adj_list[0].push_back(1);
    adj_list[0].push_back(2);
    adj_list[0].push_back(3);

    adj_list[1].push_back(0);
    adj_list[1].push_back(3);
    adj_list[1].push_back(6);

    adj_list[2].push_back(0);
    adj_list[2].push_back(3);
    adj_list[2].push_back(5);

    adj_list[3].push_back(0);
    adj_list[3].push_back(1);
    adj_list[3].push_back(2);
    adj_list[3].push_back(4);
    adj_list[3].push_back(5);
    adj_list[3].push_back(6);
    adj_list[3].push_back(7);

    adj_list[4].push_back(3);
    adj_list[4].push_back(6);

    adj_list[5].push_back(2);
    adj_list[5].push_back(3);
    adj_list[5].push_back(6);
    adj_list[5].push_back(7);

    adj_list[6].push_back(1);
    adj_list[6].push_back(3);
    adj_list[6].push_back(4);
    adj_list[6].push_back(5);

    adj_list[7].push_back(3);
    adj_list[7].push_back(5);

    GraphicMatroid graph(adj_list);
    graph.GenerateKForests(2);
    auto forests = graph.GetForests();

    graph.PrintGraph();

    for (const auto& forest : forests) {
        for (auto edge : forest) {
            std::cout << edge.first << " " << edge.second << "\n";
        }
        std::cout << "\n";
    }

    return 0;
}
