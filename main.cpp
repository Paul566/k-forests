#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <filesystem>
#include "GraphicMatroid.h"

std::vector<std::vector<int>> ReadAdjList(const std::string &path) {
    std::vector<std::vector<int>> adj_list;
    std::fstream input_file(path);

    if (input_file.is_open()) {
        std::string line;
        std::getline(input_file, line);

        int num_vertices, num_edges;
        std::istringstream stream(line);
        stream >> num_vertices >> num_edges;

        for (int i = 0; i < num_vertices; ++i) {
            std::getline(input_file, line);
            std::vector<int> neighbors;

            stream = std::istringstream(line);
            int next_vertex;
            while (stream >> next_vertex) {
                neighbors.push_back(next_vertex - 1);
            }

            adj_list.push_back(neighbors);
        }
    } else {
        throw std::runtime_error("input file not found");
    }
    input_file.close();

    return adj_list;
}

int main() {
    std::string prefix = std::filesystem::current_path().string() + "/../tests/random-graphs/";

    auto adj_list = ReadAdjList(prefix + "3-3.txt");

    GraphicMatroid graph(adj_list);
    graph.PrintGraph();

    graph.GenerateKForests(2);
    auto forests = graph.GetForests();
    graph.PrintGraph();


    return 0;
}
