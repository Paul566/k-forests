#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <filesystem>
#include "GraphicMatroid.h"
#include "Tester.h"
#include "DisjointSets.h"


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
        throw std::runtime_error("input file " + path + " not found");
    }
    input_file.close();

    return adj_list;
}

std::vector<int> ReadAnswers(const std::string &path) {
    std::vector<int> answers;
    std::fstream input_file(path);

    if (input_file.is_open()) {
        int next_answer;
        while (input_file >> next_answer) {
            answers.push_back(next_answer);
        }
    } else {
        throw std::runtime_error("input file " + path + " not found");
    }

    input_file.close();

    return answers;
}

std::vector<int> SolveForAllK(const std::vector<std::vector<int>> &adj_list) {
    // returns (a_0, a_1, ..., a_n), where a_k is the size of the optimal solution for k = k
    std::vector<int> answers;

    for (int k = 0; k <= static_cast<int>(adj_list.size()); ++k) {
        GraphicMatroid graph(adj_list);
        graph.GenerateKForests(k);
        auto forests = graph.GetForests();
        int optimal_size = 0;
        for (const auto &forest: forests) {
            optimal_size += static_cast<int>(forest.size());
        }
        answers.push_back(optimal_size);
    }

    return answers;
}

void ExportVector(const std::string &path, const std::vector<int> &vector) {
    std::ofstream output(path);

    for (int element: vector) {
        output << element << "\n";
    }

    output.close();
}

void RunRandomTests() {
    std::string prefix = std::filesystem::current_path().string() + "/../tests/random-graphs/";

    std::cout << std::fixed;
    std::cout << std::setprecision(4);

    for (int n = 3; n <= 15; ++n) {
        for (int m = n; m <= n * (n - 1) / 2; ++m) {
            std::string filename_graph = std::to_string(n) + "-" + std::to_string(m) + ".txt";
            std::string filename_answers = std::to_string(n) + "-" + std::to_string(m) + "-answers.txt";

            std::cout << filename_graph << ":" << std::endl;

            auto adj_list = ReadAdjList(prefix + filename_graph);
            auto answers = ReadAnswers(prefix + filename_answers);

            Tester tester(adj_list, answers);
            auto times = tester.RunForAllK();

            for (double time: times) {
                std::cout << time << " ";
            }
            std::cout << "\n";
        }
    }

    std::cout << "\nAll tests passed!\n";
}

int main() {

    RunRandomTests();

    /*std::string prefix = std::filesystem::current_path().string() + "/../tests/random-graphs/";

    std::string filename_graph = "8-21.txt";
    std::string filename_answers = "8-21-answers.txt";

    auto adj_list = ReadAdjList(prefix + filename_graph);
    auto answers = ReadAnswers(prefix + filename_answers);

    GraphicMatroid graph(adj_list);
    graph.GenerateKForests(3);
    graph.PrintGraph();

    auto forests = graph.GetForests();
    for (const auto& forest : forests) {
        std::cout << forest.size() << " ";
    }*/

    return 0;
}
