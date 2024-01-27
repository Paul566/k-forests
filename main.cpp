#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <filesystem>
#include "GraphicMatroid.h"
#include "Tester.h"

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

std::vector<int> ReadAnswers(const std::string& path) {
    std::vector<int> answers;
    std::fstream input_file(path);
    int next_answer;
    while (input_file >> next_answer) {
        answers.push_back(next_answer);
    }
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

void ExportAnswers(const std::string &path, const std::vector<int> &answers) {
    std::ofstream output(path);

    for (int answer: answers) {
        output << answer << "\n";
    }

    output.close();
}

void TestAllK(const std::vector<std::vector<int>> &adj_list, const std::vector<int> &ground_truth_answers) {

}

int main() {
    std::string prefix = std::filesystem::current_path().string() + "/../tests/random-graphs/";

    auto adj_list = ReadAdjList(prefix + "12-37.txt");
    auto answers = ReadAnswers(prefix + "12-37-answers.txt");

    Tester tester(adj_list, answers);
    auto times = tester.RunForAllK();

    for (double time : times) {
        std::cout << time << "\n";
    }

    /*for (int n = 3; n <= 15; ++n) {
        for (int m = n; m <= n * (n - 1) / 2; ++m) {
            auto adj_list = ReadAdjList(prefix + std::to_string(n) + "-" + std::to_string(m) + ".txt");
            auto answers = SolveForAllK(adj_list);
            ExportAnswers(prefix + std::to_string(n) + "-" + std::to_string(m) + "-answers.txt", answers);

            std::cout << "\n\n" << std::to_string(n) + "-" + std::to_string(m) + ".txt\n";
            for (int answer: answers) {
                std::cout << answer << " ";
            }
        }
    }*/

    return 0;
}
